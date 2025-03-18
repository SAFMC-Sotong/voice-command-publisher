
import pyaudio
import socket
import numpy as np
import time
import struct
import argparse

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024 * 2
SILENCE_THRESHOLD = 1000
SILENCE_DURATION = 0.7
MIN_SPEECH_DURATION = 0.3
MAX_SPEECH_DURATION = 5.0  # in seconds

def is_speech(audio_data):
    """Detect if audio chunk contains speech based on energy threshold"""
    if isinstance(audio_data, bytes):
        audio_data = np.frombuffer(audio_data, dtype=np.int16)
    energy = np.sqrt(np.mean(np.square(audio_data.astype(np.float32))))
    return energy > SILENCE_THRESHOLD

def send_transcription_to_vc(transcription):
    """Send the transcription result to the voice control process using a UNIX domain socket."""
    UNIX_SOCKET_PATH = "/tmp/voice_control.sock"
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.connect(UNIX_SOCKET_PATH)
            s.sendall(transcription.encode('utf-8'))
    except Exception as e:
        print(f"Error sending transcription to voice control: {e}")

def main(server_ip, server_port):
    # Initialize PyAudio
    audio = pyaudio.PyAudio()
    
    # Setup audio input stream
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)
    
    # Setup socket connection to the remote server (still using TCP)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # Connect to remote server
        print(f"Connecting to server at {server_ip}:{server_port}...")
        client_socket.connect((server_ip, server_port))
        print("Connected successfully!")
        
        # Send audio parameters to the server
        params = struct.pack('!IIIII', RATE, CHANNELS, CHUNK, 
                             int(SILENCE_DURATION * 1000), 
                             int(MIN_SPEECH_DURATION * 1000))
        client_socket.sendall(params)
        
        # Stream audio to the server
        print("Streaming audio to server. Press Ctrl+C to stop.")
        
        # Variables for speech detection
        buffer = b''
        speech_active = False
        silence_frames = 0
        required_silence_frames = int(SILENCE_DURATION * RATE / CHUNK)
        min_speech_frames = int(MIN_SPEECH_DURATION * RATE / CHUNK)
        max_speech_frames = int(MAX_SPEECH_DURATION * RATE / CHUNK)
        speech_frames = 0
        
        while True:
            # Read audio chunk
            data = stream.read(CHUNK, exception_on_overflow=False)
            
            # Local speech detection
            chunk_has_speech = is_speech(data)
            buffer += data
            
            if chunk_has_speech:
                if not speech_active:
                    # Start of speech detected
                    speech_active = True
                    client_socket.sendall(b'START')
                    print("Speech detected")
                silence_frames = 0
                speech_frames += 1
                client_socket.sendall(struct.pack('!I', len(data)) + data)
            elif speech_active:
                silence_frames += 1
                client_socket.sendall(struct.pack('!I', len(data)) + data)
                
                # End utterance if speech is too long
                if speech_frames >= max_speech_frames:
                    print(f"Max speech duration reached ({MAX_SPEECH_DURATION}s)")
                    client_socket.sendall(b'ENDUTT')
                    buffer = b''
                    speech_active = False
                    speech_frames = 0
                    silence_frames = 0
                    
                    size_bytes = client_socket.recv(4)
                    if size_bytes:
                        msg_size = struct.unpack('!I', size_bytes)[0]
                        result = client_socket.recv(msg_size).decode('utf-8')
                        send_transcription_to_vc(result)
                    continue
                
                if silence_frames >= required_silence_frames and speech_frames >= min_speech_frames:
                    print(f"Silence detected ({SILENCE_DURATION}s)")
                    client_socket.sendall(b'ENDUTT')
                    buffer = b''
                    speech_active = False
                    speech_frames = 0
                    silence_frames = 0
                    
                    size_bytes = client_socket.recv(4)
                    if size_bytes:
                        msg_size = struct.unpack('!I', size_bytes)[0]
                        result = client_socket.recv(msg_size).decode('utf-8')
                        send_transcription_to_vc(result)
            
            # Prevent buffer overflow
            max_buffer_size = 15 * RATE * 2
            if len(buffer) > max_buffer_size:
                buffer = buffer[-max_buffer_size:]
                
    except KeyboardInterrupt:
        print("Stopping audio streaming...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stream.stop_stream()
        stream.close()
        audio.terminate()
        client_socket.close()
        print("Client terminated")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Audio streaming client for speech recognition")
    parser.add_argument("--server", type=str, default="10.100.26.221", 
                        help="Server IP address")
    parser.add_argument("--port", type=int, default=12345, 
                        help="Server port")
    args = parser.parse_args()
    
    main(args.server, args.port)
