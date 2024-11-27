// // Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0

#ifndef B_MANIPULATED_RVIZ_UI__MT_RVIZ_UI_HPP_
#define B_MANIPULATED_RVIZ_UI__MT_RVIZ_UI_HPP_

#include <memory>
#include <QLineEdit>                       // For text input fields in the GUI.
#include <QPushButton>                     // For buttons in the GUI.
#include <QVBoxLayout>                     // For organizing layout.
#include <rviz_common/panel.hpp>           // Base class for creating a panel in RViz.
#include <tf2_ros/transform_broadcaster.h> // Sends transformation data in ROS.
#include <rclcpp/rclcpp.hpp>               // ROS2 node operation.

namespace b_manipulated_rviz_ui
{
    class MTRvizUI : public rviz_common::Panel
    {
        Q_OBJECT                                          // A special macro that allows the class to use Qt's signal-slot system (how buttons and other events communicate).
            public : MTRvizUI(QWidget *parent = nullptr); // Constructor: sets up the panel.
        virtual void onInitialize() override;             // Called when the panel is initialized.

    private Q_SLOTS:
        // Publishes the transform for the entered frame name.
        void publishFrame();

    private:
        // // Sets the status and logs messages.
        // void setStatus(const std::string &status, const std::string &name, const std::string &message);

        // ROS 2 node for communication.
        rclcpp::Node::SharedPtr node_;
        // Transform broadcaster for publishing frames.
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Text input fields for frame name and parent frame.
        QLineEdit *frame_name_input_;
        QLineEdit *parent_frame_input_;
        // Publish button.
        QPushButton *publish_button_;
    };
} // namespace b_manipulated_rviz_ui

#endif // B_MANIPULATED_RVIZ_UI__MT_RVIZ_UI_HPP_
