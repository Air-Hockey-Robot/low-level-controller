from AirHockeyTable import AirHockeyTable

controller = AirHockeyTable("COM6", "COM5")

if __name__ == '__main__':
    controller.log_current_state()
    controller.zero_encoders()
    controller.log_current_state()

    # controller.command_position(0.05, 0) # move 5 cm to the left
    # controller.log_current_state()

