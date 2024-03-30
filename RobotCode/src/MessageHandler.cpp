#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>

#include <Arduino.h>

#include <Robot.hpp>


MessageReceiver messageReceiver;
MessageReceiverUDP messageReceiverUDP;

void task_messageReceiver(void* parameters)
{
    Robot* robot = Robot::getInstance();
    for (;;)
    {
        Serial.println(">> TCP receiver standby...");
        MessageType mt = messageReceiver.receive();

        if (mt == MessageType::NEW_TRAJECTORY)
        {
            Serial.println("Received trajectory, following...");
            robot->motionController->resetPosition(messageReceiver.targetTrajectory.getCurrentPoint(0).position, true, true, true);
            robot->movement_override = true;
            robot->motionController->setTrajectoryToFollow(messageReceiver.targetTrajectory);
            robot->motionController->waitForTrajectoryFinished();
            robot->movement_override = false;
        }
        // else if (mt == MessageType::SET_ID)
        // {
        //     Serial.print("Received new id: ");
        //     Serial.println(messageReceiver.newID);
        //     robot->robotID = messageReceiver.newID;
        //     robot->preferences.putInt("id", robot->robotID);
        // }
        // else if (mt == MessageType::NEW_TRAJECTORY_SAVE)
        // {
        // Serial.println("Received trajectory, saving...");

        // robot->length_of_saved_traj_float = (int)messageReceiver.receivedTrajectory.at(1) * 5;
        // robot->duration_of_saved_traj = (float)messageReceiver.receivedTrajectory.at(2);

        // if ((messageReceiver.receivedTrajectory.size() - 3) == robot->length_of_saved_traj_float)
        // {
        //     robot->preferences.putInt("traj_len_float", robot->length_of_saved_traj_float);
        //     robot->preferences.putFloat("traj_duration", robot->duration_of_saved_traj);

        //     // Ignore first 3 bytes
        //     float* tmp = new float[robot->length_of_saved_traj_float]();
        //     for (int i = 0; i < robot->length_of_saved_traj_float; i++)
        //     {
        //         tmp[i] = messageReceiver.receivedTrajectory.at(i+3);
        //     }
        //     robot->preferences.putBytes("traj_coord", tmp, robot->length_of_saved_traj_float * 4);
        //     Serial.print("Saved traj of length ");
        //     Serial.println(robot->length_of_saved_traj_float);

        //     delete tmp;
        // }
        // else
        // {
        //     Serial.println("Not saving: decrepency in sizes");
        //     Serial.print("Expected ");
        //     Serial.print(robot->length_of_saved_traj_float);
        //     Serial.print(" received ");
        //     Serial.println(messageReceiver.receivedTrajectory.size() - 3);
        // }
        // }
        else if (mt == MessageType::MATCH_STATE)
        {
        Serial.println("Received match state");
        if (messageReceiver.matchStarted)
        {
            Serial.print("Match started, current time ");
            Serial.println(messageReceiver.matchCurrentTime);
            robot->match_started = true;
            robot->trajectory_was_read = false;
            robot->match_current_time_s = messageReceiver.matchCurrentTime;
        }
        else
        {
            Serial.println("Match not started");
            robot->match_started = false;
            robot->match_current_time_s = 0.0f;
            robot->trajectory_was_read = false;
            robot->motionController->clearTrajectories();
        }
        }
        else
        {
        Serial.println("Received error");
        }
    }
}

void task_messageReceiverUDP(void* parameters)
{
    Robot* robot = Robot::getInstance();
    for (;;)
    {
        Serial.println(">> UDP receiver standby...");
        MessageType mt = messageReceiverUDP.receive();

        if (mt == MessageType::MATCH_STATE)
        {
        Serial.println("Received match state");
        if (messageReceiverUDP.matchStarted)
        {
            Serial.print("Match started, current time ");
            Serial.println(messageReceiverUDP.matchCurrentTime);
            robot->match_started = true;
            robot->trajectory_was_read = false;
            robot->match_current_time_s = messageReceiverUDP.matchCurrentTime;
        }
        else
        {
            Serial.println("Match not started");
            robot->match_started = false;
            robot->match_current_time_s = 0.0f;
            robot->trajectory_was_read = false;
            robot->motionController->clearTrajectories();
        }
        }
        else
        {
        Serial.println("Received error");
        }
    }
}


namespace MessageHandler{

    void start()
    {
        messageReceiver.begin();
        messageReceiverUDP.begin();

        xTaskCreate(
            task_messageReceiver,
            "task_messageReceiver",
            50000,
            NULL,
            40,
            NULL
        );

        xTaskCreate(
            task_messageReceiverUDP,
            "task_messageReceiverUDP",
            50000,
            NULL,
            40,
            NULL
        );
    }
}