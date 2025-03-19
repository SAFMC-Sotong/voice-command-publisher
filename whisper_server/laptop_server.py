
from faster_whisper import WhisperModel
import numpy as np
import socket
import struct
import threading
import time
import argparse
import os

# Use environment variable for model cache if needed
os.environ["FASTER_WHISPER_CACHE"] = os.path.expanduser("/home/ubuntu/Documents/ws_ros2/src/faster-whisper/model")

def process_audio_chunk(model, audio_data, rate):
    """Process audio data with Whisper model"""
    # Convert buffer to numpy array
    audio_float = np.frombuffer(audio_data, np.int16).astype(np.float32) / 32768.0
    
    # Measure processing time
    start_time = time.time()
    
    # Use optimized settings for real-time transcription
    segments, info = model.transcribe(
        audio_float, 
        language="en",
        beam_size=1,
        best_of=1,
        temperature=0.0,
        compression_ratio_threshold=2.4,
        condition_on_previous_text=False,
        no_speech_threshold=0.6,
        word_timestamps=False
    )
    
    # Process results
    transcript = ""
    for segment in segments:
        transcript += segment.text + " "
    
    # Calculate processing time
    processing_time = time.time() - start_time
    audio_duration = len(audio_float) / rate
    rtf = processing_time / audio_duration if audio_duration > 0 else 0
    
    print(f"Transcript: {transcript.strip()}" , flush=True)
    # print(f"Processing time: {processing_time:.2f}s, Audio duration: {audio_duration:.2f}s, RTF: {rtf:.2f}x")
    
    return transcript.strip()

def handle_client(client_socket, client_address, model):
    """Handle individual client connection"""
    print(f"Connected to client: {client_address}")
    
    try:
        # Receive audio parameters
        params_data = client_socket.recv(20)
        rate, channels, chunk_size, silence_duration_ms, min_speech_duration_ms = struct.unpack('!IIIII', params_data)
        
        # print(f"Received parameters: Rate={rate}, Channels={channels}")
        
        # Buffer for collecting audio chunks
        audio_buffer = bytearray()
        
        while True:
            # Read first 4 bytes to determine what's next
            header = client_socket.recv(4)
            
            if not header:  # Connection closed
                print("Connection closed")
                break
            
            # Check for control commands
            if header == b'STAR':  # First 4 bytes of START
                # Read the last byte of START
                t_byte = client_socket.recv(1)
                if t_byte == b'T':
                    # print("Speech started...")
                    audio_buffer = bytearray()  # Reset buffer
                continue
                
            if header == b'ENDU':  # First 4 bytes of ENDUTT
                # Read the last 2 bytes of ENDUTT
                tt_bytes = client_socket.recv(2)
                if tt_bytes == b'TT':
                    # print("End of utterance, processing...")
                    if len(audio_buffer) > 0:
                        # Process the complete utterance
                        transcript = process_audio_chunk(model, audio_buffer, rate)
                        
                        # Send result back to client
                        result_bytes = transcript.encode('utf-8')
                        client_socket.sendall(struct.pack('!I', len(result_bytes)) + result_bytes)
                        
                        # Reset buffer
                        audio_buffer = bytearray()
                        

                continue
            
            # If not a control command, treat as data size
            try:
                data_size = struct.unpack('!I', header)[0]
                
                # Get the audio data
                data = bytearray()
                remaining = data_size
                
                # Ensure we get all the data
                while remaining > 0:
                    chunk = client_socket.recv(min(remaining, 4096))
                    if not chunk:
                        # print("Connection lost while receiving data")
                        break
                    data.extend(chunk)
                    remaining -= len(chunk)
                
                # Add to buffer
                audio_buffer.extend(data)
                
            except struct.error:
                print(f"Error unpacking data header")
                continue
                
    except Exception as e:
        print(f"Error handling client {client_address}: {e}")
    finally:
        client_socket.close()
        print(f"Connection closed with {client_address}")

def main(host, port, model_path, device, compute_type):
    """Main server function"""
    # Load local fine-tuned Whisper model
    print(f"Loading fine-tuned Whisper model from {model_path} (device={device}, compute_type={compute_type})...")
    model = WhisperModel(
        model_path, 
        device=device, 
        compute_type=compute_type, 
        cpu_threads=4 if device == "cpu" else 1,
        num_workers=2
    )
    
    # Pre-warm the model
    # print("Warming up model...")
    dummy_audio = np.zeros(16000, dtype=np.float32)
    model.transcribe(dummy_audio, language="en", beam_size=1)
    # print("Model ready!")
    
    # Create socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((host, port))
        server_socket.listen(5)
        # print(f"Server listening on {host}:{port}")
        
        while True:
            client_socket, client_address = server_socket.accept()
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, client_address, model)
            )
            client_thread.daemon = True
            client_thread.start()
            
    except KeyboardInterrupt:
        print("Server shutting down...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()
        print("Server terminated")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Audio processing server for speech recognition")
    parser.add_argument("--host", type=str, default="0.0.0.0", 
                        help="Server host address")
    parser.add_argument("--port", type=int, default=12345, 
                        help="Server port")
    parser.add_argument("--model-path", type=str, default="/home/ubuntu/Downloads/drone_model", 
                        help="Path to local fine-tuned model")
    parser.add_argument("--device", type=str, default="cpu", 
                        choices=["cpu", "cuda"], 
                        help="Device to run model on")
    parser.add_argument("--compute", type=str, default="int8", 
                        choices=["int8", "int8_float16", "float16", "float32"], 
                        help="Compute type")
    args = parser.parse_args()
    
    main(args.host, args.port, args.model_path, args.device, args.compute)

    # Example usage:
    # python3 tests/laptop_server.py --host 0.0.0.0 --port 12345 --model-path /home/hn/Downloads/drone_model --device cpu --compute int8