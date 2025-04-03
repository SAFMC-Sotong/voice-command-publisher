#include <iostream>
#include <string>
#include <thread>
#include <regex>
#include <algorithm>
#include <cstring>
#include <cmath>  // Added this for M_PI
#include <unistd.h>
#include <arpa/inet.h>
#include <wiringPi.h>
#include <sys/socket.h>
#include <sys/un.h> // For UNIX domain sockets

// Define M_PI if it's not defined in cmath
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    
    // Patterns for numerical commands
    std::regex move_forward_pattern_ = std::regex("(?:move|go) forward (\\d+(?:\\.\\d+)?)\\s*(?:meter|meters|m)?");
    std::regex move_backward_pattern_ = std::regex("(?:move|go) backward (\\d+(?:\\.\\d+)?)\\s*(?:meter|meters|m)?");
    std::regex go_up_pattern_ = std::regex("(?:go|move) up (\\d+(?:\\.\\d+)?)\\s*(?:meter|meters|m)?");
    std::regex go_down_pattern_ = std::regex("(?:go|move) down (\\d+(?:\\.\\d+)?)\\s*(?:meter|meters|m)?");
    std::regex yaw_left_pattern_ = std::regex("(?:yaw|turn) left (\\d+(?:\\.\\d+)?)\\s*(?:degree|degrees|deg)?");
    std::regex yaw_right_pattern_ = std::regex("(?:yaw|turn) right (\\d+(?:\\.\\d+)?)\\s*(?:degree|degrees|deg)?");
    std::regex roll_right_pattern_ = std::regex("(?:move|roll|go) right (\\d+(?:\\.\\d+)?)\\s*(?:degree|degrees|deg)?");
    std::regex roll_left_pattern_ = std::regex("(?:move|roll|go) left (\\d+(?:\\.\\d+)?)\\s*(?:degree|degrees|deg)?");




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
            std::cout << "Sent command to " << active_drone_ << ": " << cmd.command 
                      << " with params: " << cmd.param1 << ", " << cmd.param2 << std::endl;
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
        // lower_text.erase(std::remove(lower_text.begin(), lower_text.end(), '.'), lower_text.end());

        // std::cout << "Now controlling: Drone Beta" << lower_text << std::endl;

        // Change active drone based on keywords
        if (lower_text.find("drone alpha") != std::string::npos) {
            active_drone_ = "alpha";
            std::cout << "Now controlling: Drone Alpha" << std::endl;
        } else if (lower_text.find("drone beta") != std::string::npos) {
            active_drone_ = "beta";
            std::cout << "Now controlling: Drone Beta" << std::endl;
        }
        
        // Process numerical commands
        std::smatch match;
        if (std::regex_search(lower_text, match, move_forward_pattern_)) {
            float distance = std::stof(match[1]);
            move_forward(distance);
        } else if (std::regex_search(lower_text, match, move_backward_pattern_)) {
            float distance = std::stof(match[1]);
            move_backward(distance);   
        } else if (std::regex_search(lower_text, match, roll_left_pattern_)) {
            float distance = std::stof(match[1]);
            move_left(distance);  
        } else if (std::regex_search(lower_text, match, roll_right_pattern_)) {
            float distance = std::stof(match[1]);
            move_right(distance);  
        } else if (std::regex_search(lower_text, match, go_up_pattern_)) {
            float distance = std::stof(match[1]);
            throttle_up(distance);
        } else if (std::regex_search(lower_text, match, go_down_pattern_)) {
            float distance = std::stof(match[1]);
            throttle_down(distance);
        } else if (std::regex_search(lower_text, match, yaw_left_pattern_)) {
            float angle = std::stof(match[1]);
            yaw_left(angle);
        } else if (std::regex_search(lower_text, match, yaw_right_pattern_)) {
            float angle = std::stof(match[1]);
            yaw_right(angle);
        }
        // Process non-numerical commands
        else if (lower_text.find("go arm") != std::string::npos) {
            arm_drone();
        } else if (lower_text.find("go disarm") != std::string::npos) {
            disarm_drone();
        } else if (lower_text.find("off board") != std::string::npos) {
            switch_to_offboard_mode();
        } else if (lower_text.find("gripper open") != std::string::npos) {
            open_gripper();
        } else if (lower_text.find("gripper close") != std::string::npos) {
            close_gripper();
        } else if (lower_text.find("go up") != std::string::npos || 
                    lower_text.find("move up") != std::string::npos) {
            throttle_up(0.5f); // Default distance
        } else if (lower_text.find("go down") != std::string::npos || 
                    lower_text.find("move down") != std::string::npos ) {
            throttle_down(0.5f); // Default distance
        } else if (lower_text.find("turn left") != std::string::npos || 
                   lower_text.find("yaw left") != std::string::npos) {
            yaw_left(15.0f); // Default angle
        } else if (lower_text.find("turn right") != std::string::npos || 
                   lower_text.find("yaw right") != std::string::npos) {
            yaw_right(15.0f); // Default angle
        } else if (lower_text.find("move forward") != std::string::npos ||
                    lower_text.find("go forward") != std::string::npos) {
            move_forward(0.3f); // Default distance
        } else if (lower_text.find("move backward") != std::string::npos ||
                    lower_text.find("go backward") != std::string::npos) {
            move_backward(0.3f); // Default distance
         } else if (lower_text.find("move left") != std::string::npos ||
                    lower_text.find("roll left") != std::string::npos ||
                    lower_text.find("go left") != std::string::npos){
            move_left(0.3f); // Default distance
        } else if (lower_text.find("move right") != std::string::npos ||
                    lower_text.find("roll right") != std::string::npos ||
                    lower_text.find("go right") != std::string::npos) {
            move_right(0.3f); // Default distance
        } else if (lower_text.find("stop") != std::string::npos) {
            stop_movement();
        } else if (lower_text.find("go record") != std::string::npos || 
                    lower_text.find("record path") != std::string::npos) {
            start_path_recording();
        } else if (lower_text.find("reach") != std::string::npos || 
                    lower_text.find("no record") != std::string::npos) {
            stop_path_recording();
        } else if (lower_text.find("go back") != std::string::npos || 
                    lower_text.find("return path") != std::string::npos || 
                    lower_text.find("return home") != std::string::npos) {
            return_path();
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

    void throttle_up(float distance = 0.5f) {
        VehicleCommand cmd = {178, distance , -1.0f, 0, 0, 0, 0, true};
        std::cout << "Throttle up " << distance << " meters" << std::endl;
        send_udp(cmd);
    }

    void throttle_down(float distance = 0.5f) {
        VehicleCommand cmd = {178, distance , 1.0f, 0, 0, 0, 0, true};
        std::cout << "Throttle down " << distance << " meters" << std::endl;
        send_udp(cmd);
    }

    void yaw_left(float angle = 15.0f) {
        // Convert to radians if needed by your flight controller
        // float angle_rad = angle * (M_PI / 180.0f);
        VehicleCommand cmd = {179, -1.0f, angle, 0, 0, 0, 0, true};
        std::cout << "Yaw left " << angle << " degrees" << std::endl;
        send_udp(cmd);
    }

    void yaw_right(float angle = 15.0f) {
        // Convert to radians if needed by your flight controller
        // float angle_rad = angle * (M_PI / 180.0f);
        VehicleCommand cmd = {179, 1.0f, angle, 0, 0, 0, 0, true};
        std::cout << "Yaw right " << angle << " degrees" << std::endl;
        send_udp(cmd);
    }

    void move_forward(float distance = 0.3f) {
        VehicleCommand cmd = {180, distance, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Move forward " << distance << " meters" << std::endl;
        send_udp(cmd);
    }
    
    void move_backward(float distance = 0.3f) {
        VehicleCommand cmd = {180, -distance, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Move backward " << distance << " meters" << std::endl;
        send_udp(cmd);
    }

    void move_left(float distance = 0.3f) {
        VehicleCommand cmd = {184, distance, -1.0f, 0, 0, 0, 0, true};
        std::cout << "Move left " << distance << " meters" << std::endl;
        send_udp(cmd);
    }

    void move_right(float distance = 0.3f) {
        VehicleCommand cmd = {184, distance, 1.0f, 0, 0, 0, 0, true};
        std::cout << "Move right " << distance << " meters" << std::endl;
        send_udp(cmd);
    }

    void stop_movement() {
        VehicleCommand cmd = {181, 0.0f, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Stop movement" << std::endl;
        send_udp(cmd);
    }

    // Add these new member functions to the VoiceControl class
    void start_path_recording() {
        VehicleCommand cmd = {182, 1.0f, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Starting path recording" << std::endl;
        send_udp(cmd);
    }

    void stop_path_recording() {
        VehicleCommand cmd = {182, 0.0f, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Stopping path recording - destination reached" << std::endl;
        send_udp(cmd);
    }

    void return_path() {
        VehicleCommand cmd = {183, 1.0f, 0.0f, 0, 0, 0, 0, true};
        std::cout << "Returning along recorded path" << std::endl;
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
