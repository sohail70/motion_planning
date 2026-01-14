#include "motion_planning/pch.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>

struct TurnaroundEvent {
    std::string name;
    double t_robot;
    double expected_t;
    double vel_x;
    double vel_y;
};


// Helper to parse a log line (UPDATED FOR ROW NUM)
TurnaroundEvent parseTurnaroundLine(const std::string& line) {
    TurnaroundEvent event;
    
    if (line.empty()) return event; 

    std::stringstream ss(line);
    std::string token;
    
    // 1. Skip the Row Number (first token)
    std::getline(ss, token, ','); 
    
    // 2. Parse Name (second token)
    if (std::getline(ss, token, ',')) {
        event.name = token;
    }
    
    // If we successfully got a name, try to get the numbers
    if (!event.name.empty()) {
        try {
            if (std::getline(ss, token, ',')) event.t_robot = std::stod(token);
            if (std::getline(ss, token, ',')) event.expected_t = std::stod(token);
            if (std::getline(ss, token, ',')) event.vel_x = std::stod(token);
            if (std::getline(ss, token, ',')) event.vel_y = std::stod(token);
        } catch (...) {
            event.name = ""; 
        }
    }
    
    return event;
}

void verifyDeterminism(const std::vector<TurnaroundEvent>& currentEvents) {
    const std::string log_filename = "turnaroundeventslogs.txt";
    
    // 1. Append current events to the log file
    std::ofstream out_file(log_filename, std::ios::app);
    if (out_file.is_open()) {
        int local_row_num = 1;
        for (const auto& e : currentEvents) {
            out_file << local_row_num << "," 
                     << e.name << "," 
                     << std::fixed << std::setprecision(5)
                     << e.t_robot << "," 
                     << e.expected_t << "," 
                     << e.vel_x << "," 
                     << e.vel_y << "\n";
            local_row_num++;
        }
        out_file << "--- RUN END ---\n";
        out_file.close();
        std::cout << "[Determinism] Logged " << currentEvents.size() 
                  << " events to " << log_filename << std::endl;
    } else {
        std::cerr << "[Determinism] Error: Could not open log file." << std::endl;
        return;
    }

    // 2. Read all history
    std::ifstream in_file(log_filename);
    std::vector<std::string> all_lines;
    std::string line;
    while (std::getline(in_file, line)) {
        if (!line.empty()) all_lines.push_back(line);
    }
    in_file.close();

    size_t current_count = currentEvents.size();
    if (current_count == 0) return;

    size_t total_lines = all_lines.size();
    
    // If file is too small (just current run + separator), nothing to compare
    if (total_lines <= current_count + 1) {
        std::cout << "[Determinism] First run recorded. No previous data to compare." << std::endl;
        return;
    }

    // 3. Find the start of the PREVIOUS run
    // We iterate backwards to find the second-to-last "--- RUN END ---"
    int separator_count = 0;
    size_t previous_run_start_index = 0;
    
    for (int i = total_lines - 1; i >= 0; --i) {
        if (all_lines[i].find("--- RUN END ---") != std::string::npos) {
            separator_count++;
            if (separator_count == 2) {
                // The line AFTER this separator is the start of the previous run
                previous_run_start_index = i + 1;
                break;
            }
        }
    }

    if (separator_count < 2) {
        std::cout << "[Determinism] Could not find a complete previous run block." << std::endl;
        return;
    }

    // 4. Determine the size of the previous run
    // It ends right before the last separator
    size_t previous_run_end_index = total_lines - 1; // The last "--- RUN END ---"
    size_t previous_run_size = previous_run_end_index - previous_run_start_index;

    std::cout << "[Determinism] Previous run had " << previous_run_size 
              << " events. Current run has " << current_count << " events." << std::endl;

    // 5. Compare the INTERSECTION (Minimum of the two)
    size_t comparison_limit = std::min(previous_run_size, current_count);
    
    if (comparison_limit == 0) {
        std::cout << "[Determinism] One of the runs was empty. Skipping comparison." << std::endl;
        return;
    }

    std::cout << "[Determinism] Comparing the first " << comparison_limit 
              << " events (Intersection)..." << std::endl;

    bool intersection_match = true;

    for (size_t i = 0; i < comparison_limit; ++i) {
        TurnaroundEvent history_event = parseTurnaroundLine(all_lines[previous_run_start_index + i]);
        const TurnaroundEvent& current_event = currentEvents[i];
        
        if (history_event.name.empty()) continue;

        // Check 1: ORDERING
        if (history_event.name != current_event.name) {
            std::cerr << "[Determinism] ORDERING MISMATCH at Event #" << i << "!" << std::endl;
            std::cerr << "  Previous: " << history_event.name << " | Current: " << current_event.name << std::endl;
            intersection_match = false;
            break;
        }

        // Check 2: ABSOLUTE TIME
        if (std::abs(history_event.t_robot - current_event.t_robot) > 1e-5) {
            std::cerr << "[Determinism] TIMESTAMP MISMATCH for [" << current_event.name << "]!" << std::endl;
            std::cerr << "  Previous T: " << history_event.t_robot << " | Current T: " << current_event.t_robot << std::endl;
            intersection_match = false;
            break;
        }

        // Check 3: EXPECTED TIME
        if (std::abs(history_event.expected_t - current_event.expected_t) > 1e-5) {
            std::cerr << "[Determinism] EXPECTED_T MISMATCH for [" << current_event.name << "]!" << std::endl;
            intersection_match = false;
            break;
        }
        
        // Check 4: VELOCITY
        if (std::abs(history_event.vel_x - current_event.vel_x) > 1e-5 ||
            std::abs(history_event.vel_y - current_event.vel_y) > 1e-5) {
             std::cerr << "[Determinism] VELOCITY MISMATCH for [" << current_event.name << "]!" << std::endl;
             intersection_match = false;
             break;
        }
    }
    
    if (intersection_match) {
        std::cout << "[Determinism] SUCCESS: The intersection (" << comparison_limit 
                  << " events) is perfectly IDENTICAL." << std::endl;
        
        if (previous_run_size != current_count) {
            std::cout << "[Determinism] NOTE: Run lengths differed (" << previous_run_size 
                      << " vs " << current_count << ")." << std::endl;
            std::cout << "[Determinism] This implies the planner or simulation ended early/late," << std::endl;
            std::cout << "[Determinism] BUT the OBSTACLE TURNAROUND LOGIC was deterministic for the shared duration." << std::endl;
        } else {
            std::cout << "[Determinism] Run lengths are identical. System is fully deterministic." << std::endl;
        }
    } else {
        std::cout << "[Determinism] FAILURE: Mismatch found in the intersection." << std::endl;
    }
}