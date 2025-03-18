#include <iostream>
#include <string>
#include <thread>
#include <regex>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <wiringPi.h>
#include <sys/socket.h>
#include <sys/un.h> // For UNIX domain sockets

#define BUTTON_PIN 27  // GPIO Pin for physical stop button

#pragma pack(push, 1)
struct VehicleCommand {
    uint16_t command;
    float param1;
    float param2;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;
    bool from_external;
};
#pragma pack(pop)

class VoiceControl {
public:
    VoiceControl(const std::string& udp_ip_alpha, int udp_port_alpha, 
                 const std::string& udp_ip_beta, int udp_port_beta)
        : udp_ip_alpha_(udp_ip_alpha), udp_ip_beta_(udp_ip_beta),
          udp_port_alpha_(udp_port_alpha), udp_port_beta_(udp_port_beta),
          active_drone_("alpha") // Default to Alpha
    {
        setup_udp_socket();
        setup_gpio();
        // Start the UNIX domain socket server in a separate thread
        whisper_monitor_thread_ = std::thread(&VoiceControl::monitor_whisper_output, this);
    }

    ~VoiceControl() {
        close(udp_socket_);
        if (whisper_monitor_thread_.joinable()) {
            whisper_monitor_thread_.join();
        }
    }

private:
    std::string udp_ip_alpha_, udp_ip_beta_;
    int udp_port_alpha_, udp_port_beta_;
    std::string active_drone_;
    int udp_socket_;
    std::thread whisper_monitor_thread_;
    // Adjusted regex to capture the transcription text
    std::regex heard_pattern_ = std::regex("Transcription: (.*)");

    void setup_udp_socket() {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            exit(1);
        }
    }

    void send_udp(const VehicleCommand& cmd) {
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(active_drone_ == "alpha" ? udp_port_alpha_ : udp_port_beta_);
        inet_pton(AF_INET, active_drone_ == "alpha" ? udp_ip_alpha_.c_str() : udp_ip_beta_.c_str(), &server_addr.sin_addr);


        ssize_t sent = sendto(udp_socket_, reinterpret_cast<const uint8_t*>(&cmd),sizeof(cmd), 0, 
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            std::cerr << "Failed to send UDP packet" << std::endl;
        } else {
            std::cout << "Sent command to " << active_drone_ << ": " << cmd.command << std::endl;
        }
    }

    void setup_gpio() {
        wiringPiSetupGpio();
        pinMode(BUTTON_PIN, INPUT);
        pullUpDnControl(BUTTON_PIN, PUD_UP);
        std::thread([this]() {
            while (true) {
                if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed
                    std::cout << "GPIO button pressed! Sending stop command." << std::endl;
                    stop_movement();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }).detach();
    }

    // UNIX domain socket server to receive transcriptions from the Python client
    void monitor_whisper_output() {
        int server_fd, client_fd;
        struct sockaddr_un address;
        const char* socket_path = "/tmp/voice_control.sock";
        
        // Create a UNIX domain socket
        if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
            std::cerr << "Failed to create UNIX domain socket" << std::endl;
            return;
        }
        
        // Remove any existing socket file
        unlink(socket_path);
        
        memset(&address, 0, sizeof(address));
        address.sun_family = AF_UNIX;
        strncpy(address.sun_path, socket_path, sizeof(address.sun_path) - 1);
        
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Bind failed on UNIX domain socket" << std::endl;
            close(server_fd);
            return;
        }
        
        if (listen(server_fd, 5) < 0) {
            std::cerr << "Listen failed on UNIX domain socket" << std::endl;
            close(server_fd);
            return;
        }
        
        std::cout << "Voice control listening for transcriptions on UNIX domain socket: " << socket_path << std::endl;
        
        char buffer[1024] = {0};
        while (true) {
            client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd < 0) {
                std::cerr << "Accept failed on UNIX domain socket" << std::endl;
                continue;
            }
            
            int bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0'; // Null-terminate the string
                std::string received(buffer);
                std::cout << "Received transcription: " << received << std::endl;
                process_whisper_output(received);
            }
            close(client_fd);
        }
        
        close(server_fd);
        unlink(socket_path);
    }

    void process_whisper_output(const std::string& transcription) {
        std::string lower_text = transcription;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);
        lower_text.erase(std::remove(lower_text.begin(), lower_text.end(), '.'), lower_text.end());

        // Change active drone based on keywords
        if (lower_text.find("drone alpha") != std::string::npos) {
            active_drone_ = "alpha";
            std::cout << "Now controlling: Drone Alpha" << std::endl;
        } else if (lower_text.find("drone beta") != std::string::npos) {
            active_drone_ = "beta";
            std::cout << "Now controlling: Drone Beta" << std::endl;
        }
        // Process commands
        else if (lower_text.find("go arm") != std::string::npos) {
            arm_drone();
        }else if (lower_text.find("go disarm") != std::string::npos) {
            disarm_drone();
        } else if (lower_text.find("off board") != std::string::npos) {
            switch_to_offboard_mode();
        } else if (lower_text.find("gripper open") != std::string::npos) {
            open_gripper();
        } else if (lower_text.find("gripper close") != std::string::npos) {
            close_gripper();
        } else if (lower_text.find("go up") != std::string::npos) {
            throttle_up();
        } else if (lower_text.find("go down") != std::string::npos) {
            throttle_down();
        } else if (lower_text.find("turn left") != std::string::npos) {
            yaw_left();
        } else if (lower_text.find("turn right") != std::string::npos) {
            yaw_right();
        } else if (lower_text.find("move forward") != std::string::npos) {
            move_forward();
        } else if (lower_text.find("stop") != std::string::npos) {
            stop_movement();
        }
    }

    void arm_drone() {
        VehicleCommand cmd = {400, 1.0f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void disarm_drone() {
        VehicleCommand cmd = {400, 0.0f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }
    void switch_to_offboard_mode() {
        VehicleCommand cmd = {176, 1.0f, 6.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void open_gripper() {
        VehicleCommand cmd = {187, 1.0f, 1.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void close_gripper() {
        VehicleCommand cmd = {187, -1.0f, -1.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void throttle_up() {
        VehicleCommand cmd = {178, 2.0f, -1.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void throttle_down() {
        VehicleCommand cmd = {178, 2.0f, 1.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void yaw_left() {
        VehicleCommand cmd = {179, -1.0f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void yaw_right() {
        VehicleCommand cmd = {179, 1.0f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void move_forward() {
        VehicleCommand cmd = {180, 0.3f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }

    void stop_movement() {
        VehicleCommand cmd = {181, 0.0f, 0.0f, 0, 0, 0, 0, true};
        send_udp(cmd);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] 
                  << " <udp_ip_alpha> <udp_port_alpha> <udp_ip_beta> <udp_port_beta>" 
                  << std::endl;
        return 1;
    }

    try {
        std::string udp_ip_alpha = argv[1];
        int udp_port_alpha = std::stoi(argv[2]);
        std::string udp_ip_beta = argv[3];
        int udp_port_beta = std::stoi(argv[4]);

        VoiceControl voiceControl(udp_ip_alpha, udp_port_alpha, udp_ip_beta, udp_port_beta);
        // Keep main running indefinitely.
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
