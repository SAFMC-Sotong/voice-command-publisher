
# Voice Control UDP Sender

## Overview
This project implements a UDP sender for voice control applications. The sender transmits data over a network to a specified receiver. This guide provides steps to compile and run the program on **Raspbian** or any **Linux-based system**.

---

## Steps to Compile and Run (`voice_control.cpp`)

### 1. Ensure GCC is Installed
Check if `g++` is installed by running:
```sh
g++ --version
```
If `g++` is not installed, install it using:
```sh
sudo apt update && sudo apt install g++
```

---

### 2. Save the Code
Ensure your UDP sender code is saved in a file named **`voice_control.cpp`**.

---

### 3. Compile the Code
Make sure wiringPi is installed:
```sh
# fetch the source
sudo apt install git
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi

# build the package
./build debian
mv debian-template/wiringpi_3.14_arm64.deb .

#install
sudo apt install ./wiringpi_3.14_arm64.deb 
```
Use `g++` to compile the code:
```sh
g++ -o voice_control voice_control.cpp -Wall
```
**Explanation of flags:**
- `-o voice_control`: Names the output binary as `voice_control`.
- `-Wall`: Enables all compiler warnings.

If you encounter errors related to missing libraries, try compiling with:
```sh
g++ -o voice_control voice_control.cpp -Wall -lwiringPi -lpthread
```
This ensures compatibility with the **pthread** and **wiringPi** library if needed.

---

### 4. Run the UDP Sender
Before running the sender, ensure the **UDP receiver** is running on the target device. Then, execute:
```sh
./voice_control
```

---

## Debugging and Network Monitoring

### Checking UDP Packets
To verify that packets are being sent, use `tcpdump`:
```sh
sudo tcpdump -i wlan0 udp port <PORT_NUMBER>
```
Replace `<PORT_NUMBER>` with the port your sender is using.

### Checking Open UDP Ports
To check if the receiver is listening on the correct port:
```sh
netstat -ulnp | grep <PORT_NUMBER>
```

---

## Troubleshooting

### Common Errors and Fixes

1. **Permission Denied (`./voice_control: Permission denied`)**
   - Fix: Grant execution permission:
     ```sh
     chmod +x voice_control
     ```

2. **Port Already in Use**
   - Fix: Find and terminate the process using the port:
     ```sh
     sudo lsof -i :<PORT_NUMBER>
     sudo kill -9 <PROCESS_ID>
     ```

3. **Firewall Blocking UDP Traffic**
   - Fix: Allow UDP traffic through the firewall:
     ```sh
     sudo ufw allow <PORT_NUMBER>/udp
     ```

---

## Notes
- Ensure the receiving device is **ready before sending packets**.
- Modify the source code to match your **network setup** if needed.

---

## License
This project is licensed under the **MIT License**.

## Author
Lim Kai Shan, Chang Siang Yi, Teh Jin Qian
