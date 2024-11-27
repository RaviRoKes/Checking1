// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// /* Author: Ravi */
// RViz plugin where users can input a frame name and parent frame name, then publish a transformation between them.

#include <b_manipulated_rviz_ui/mt_rviz_ui.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Qt libraries for building the user interface (layouts, text input fields, buttons, and labels).
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>

namespace b_manipulated_rviz_ui
{
    MTRvizUI::MTRvizUI(QWidget *parent)
        : rviz_common::Panel(parent),
          node_(std::make_shared<rclcpp::Node>("mt_rviz_ui_node")),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_))
    {
        // create Layout to organize UI components
        QVBoxLayout *layout = new QVBoxLayout;

        // Label and text input for frame name
        QLabel *frame_name_label = new QLabel("Frame Name:", this);
        layout->addWidget(frame_name_label);

        frame_name_input_ = new QLineEdit(this);
        frame_name_input_->setPlaceholderText("Enter frame name");
        layout->addWidget(frame_name_input_);

        // Prevent spaces in frame name input
        connect(frame_name_input_, &QLineEdit::textChanged, this, [this]()
                {
            QString text = frame_name_input_->text();
            text.replace(" ", ""); // Remove spaces
            frame_name_input_->setText(text); });

        // Label and text input for parent frame
        QLabel *parent_frame_label = new QLabel("Parent Frame:", this);
        layout->addWidget(parent_frame_label);

        parent_frame_input_ = new QLineEdit(this);
        parent_frame_input_->setPlaceholderText("Enter parent frame name");
        layout->addWidget(parent_frame_input_);

        // Publish button
        publish_button_ = new QPushButton("Publish Frame", this);
        layout->addWidget(publish_button_);

        // Connect the publish button to its callback
        connect(publish_button_, &QPushButton::clicked, this, &MTRvizUI::publishFrame);

        // Set the layout for the panel
        setLayout(layout);
    }

    void MTRvizUI::onInitialize()
    {
        // Initialize parent class
        rviz_common::Panel::onInitialize();
        // Log initialization
        RCLCPP_INFO(node_->get_logger(), "MTRvizUI plugin initialized.");
    }
    // function is triggered when the "Publish Frame" button is clicked. It publishes a transform (a coordinate frame) based on user input.
    void MTRvizUI::publishFrame()
    {
        RCLCPP_INFO(node_->get_logger(), "Publish button pressed.");
        // Retrieve values from text inputs
        std::string frame_name = frame_name_input_->text().toStdString();
        std::string parent_frame = parent_frame_input_->text().toStdString();

        if (frame_name.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Frame name cannot be empty.");
            return;
        }
        if (parent_frame.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Parent frame cannot be empty.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Creating transform: %s -> %s", parent_frame.c_str(), frame_name.c_str());
        // Additional validation: Ensure frame_name has no spaces (already handled in UI)
        // if (frame_name.find(' ') != std::string::npos)
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Frame name cannot contain spaces.");
        //     return;

            // Create a transform
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = node_->now();
            transform.header.frame_id = parent_frame;
            transform.child_frame_id = frame_name;

            // Specifies the position and orientation of the new frame. In this case, it’s at the origin with no rotation.
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 0.0;
            transform.transform.rotation.w = 1.0;

            // Publish the transform
            tf_broadcaster_->sendTransform(transform);

            // Log success
            RCLCPP_INFO(node_->get_logger(), "Published frame \"%s\" under parent frame \"%s\".",
                        frame_name.c_str(), parent_frame.c_str());
        // }

    } // namespace b_manipulated_rviz_ui
}
// Includes the pluginlib header and uses the PLUGINLIB_EXPORT_CLASS macro to register this class as an RViz plugin.
// This makes it available to be loaded as a custom panel.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(b_manipulated_rviz_ui::MTRvizUI, rviz_common::Panel)
