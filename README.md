# Wearable_device_RTOS
Designed a real-time operating system (RTOS) application for a wearable device that 
communicates with two sensors and enables data storage and retrieval over BLE

## System Design Decisions and Trade-offs
![Screenshot 2025-01-26 225913](https://github.com/user-attachments/assets/5500fdf2-59dd-439b-8a5e-b7353a627ffe)

## System Design Decisions and Trade-offs

### Use of Blocking Operations (k_sleep)
- **Decision**: Use `k_sleep` for delays.
- **Trade-off**: Can waste CPU resources by idling, which could otherwise be used for processing other tasks.

### Sequential Thread Initialization
- **Decision**: Start the `flash_thread` and `sensor_data_thread` after the BLE session starts.
- **Trade-off**: Some data may be lost during the time of thread creation.

### Basic BLE Profile Usage
- **Decision**: Use a basic BLE profile for communication.
- **Trade-off**: May not be sufficient for applications requiring complex interactions or high data throughput, potentially limiting the system's capabilities.

### Memory Alignment
- **Decision**: Align memory at multiples of 4 for faster access.
- **Trade-off**: Increases memory usage.

### Error Handling Simplicity
- **Decision**: Implement basic error handling.
- **Trade-off**: While this makes the code easier to manage and debug, it may not robustly handle all failure scenarios, leading to system instability in unexpected conditions.

### Sampling Rate
- **Decision**: Standardize the data acquisition rate at 25Hz to match the IMU sensor's output.
- **Trade-off**: This simplifies data processing and system design but underutilizes the PPG sensor's capabilities, potentially losing higher resolution data valuable in critical health monitoring scenarios.
