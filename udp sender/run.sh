#!/usr/bin/bash

source deactivate

source ~/projects/speech-to-text/client/bin/activate

nohup jackd -d alsa & 
sleep 2
python3 /home/raspberry/projects/speech-to-text/udp\ sender/pi_client.py --server 10.42.0.1 --port 12345 &
/home/raspberry/projects/speech-to-text/udp\ sender/voice_control 10.42.0.142 14551 10.42.0.47 14551


