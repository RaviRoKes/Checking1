#include "b_manipulated_rviz_ui/mt_rviz_ui.hpp"

#include <rviz_common/logging.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <interactive_markers/interactive_markers.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLineEdit>

namespace b_manipulated_rviz_ui
{

    MTRvizUI::MTRvizUI(QWidget *parent)
        : rviz_common::Panel(parent), server_(nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(this);

        QLabel *radius_label = new QLabel("Cylinder Radius:", this);
        radius_input_ = new QLineEdit(this);
        layout->addWidget(radius_label);
        layout->addWidget(radius_input_);

        QLabel *height_label = new QLabel("Cylinder Height:", this);
        height_input_ = new QLineEdit(this);
        layout->addWidget(height_label);
        layout->addWidget(height_input_);

        add_cylinder_button_ = new QPushButton("Add Cylinder", this);
        layout->addWidget(add_cylinder_button_);

        connect(add_cylinder_button_, &QPushButton::clicked, [this]()
                {
        float radius = radius_input_->text().toFloat();
        float height = height_input_->text().toFloat();
        addCylinderToGrid(radius, height); });

        // Initialize interactive markers
        createInteractiveMarkers();
    }

    MTRvizUI::~MTRvizUI()
    {
        if (server_)
        {
            delete server_;
        }
    }

    void MTRvizUI::createInteractiveMarkers()
    {
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("interactive_marker_node");

        // Fix for InteractiveMarkerServer constructor
        server_ = new interactive_markers::InteractiveMarkerServer("box_grid_server", node);

        for (int i = 0; i < 50; ++i)
        {
            visualization_msgs::msg::InteractiveMarker marker;
            marker.header.frame_id = "base_link";
            marker.name = "box_" + std::to_string(i);
            marker.description = "Box Marker " + std::to_string(i);

            visualization_msgs::msg::Marker box_marker;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.scale.x = 0.1;
            box_marker.scale.y = 0.1;
            box_marker.scale.z = 0.1;
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 1.0;
            box_marker.pose.position.x = (i % 10) * 0.2; // 10 columns
            box_marker.pose.position.y = (i / 10) * 0.2; // 5 rows
            box_marker.pose.position.z = 0.0;

            // Add control to the marker using the createInteractiveControl function
            marker.controls.push_back(createInteractiveControl(marker, box_marker));

            interactive_markers_.push_back(marker);
        }

        server_->applyChanges();
    }

    void MTRvizUI::toggleCylinderVisibility(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        // Toggle cylinder visibility based on feedback

        // Determine if the marker is being clicked or interacted with
        std::string marker_name = feedback->marker_name;

        // Iterate over interactive markers and find the corresponding marker
        for (auto &marker : interactive_markers_)
        {
            if (marker.name == marker_name)
            {
                // Check the action and toggle visibility of the cylinder accordingly
                bool is_visible = false;
                for (auto &control : marker.controls)
                {
                    for (auto &marker : control.markers)
                    {
                        // Toggle the visibility of the cylinder
                        if (marker.type == visualization_msgs::msg::Marker::CYLINDER)
                        {
                            is_visible = marker.color.a > 0.0;       // If alpha is > 0, it's visible
                            marker.color.a = is_visible ? 0.0 : 1.0; // Toggle visibility
                            break;
                        }
                    }
                }

                // Apply changes to the marker server
                server_->applyChanges();
                break;
            }
        }
    }

    void MTRvizUI::addCylinderToGrid(float radius, float height)
    {
        // Add cylinder marker to a grid position
        visualization_msgs::msg::Marker cylinder_marker;
        cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder_marker.scale.x = radius * 2; // Diameter
        cylinder_marker.scale.y = radius * 2; // Diameter
        cylinder_marker.scale.z = height;
        cylinder_marker.color.r = 0.0;
        cylinder_marker.color.g = 1.0;
        cylinder_marker.color.b = 0.0;
        cylinder_marker.color.a = 1.0;

        // Define the grid position where the cylinder should be placed
        // In this case, I'm calculating based on a simple grid (3x3 example)
        int grid_x = interactive_markers_.size() % 10; // 10 columns
        int grid_y = interactive_markers_.size() / 10; // 10 rows

        // Set the cylinder's position based on the grid
        cylinder_marker.pose.position.x = grid_x * 0.2; // x position (spacing 0.2)
        cylinder_marker.pose.position.y = grid_y * 0.2; // y position (spacing 0.2)
        cylinder_marker.pose.position.z = 0.0;          // z position (flat on the ground)

        // Create a new interactive marker for the cylinder
        visualization_msgs::msg::InteractiveMarker cylinder_interactive_marker;
        cylinder_interactive_marker.header.frame_id = "base_link";
        cylinder_interactive_marker.name = "cylinder_" + std::to_string(interactive_markers_.size());
        cylinder_interactive_marker.description = "Cylinder Marker " + std::to_string(interactive_markers_.size());

        // Add the cylinder marker as the control for the interactive marker
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        control.markers.push_back(cylinder_marker); // Attach the cylinder marker

        // Add control to the interactive marker
        cylinder_interactive_marker.controls.push_back(control);

        // Add the new interactive marker to the server
        interactive_markers_.push_back(cylinder_interactive_marker);

        // Update the interactive marker server with the new marker
        server_->insert(cylinder_interactive_marker);
        server_->applyChanges(); // Apply changes to the server
    }

    // Definition of createInteractiveControl
    visualization_msgs::msg::InteractiveMarkerControl MTRvizUI::createInteractiveControl(
        const visualization_msgs::msg::InteractiveMarker &marker,
        const visualization_msgs::msg::Marker &box_marker)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

        // Add the control to the marker's controls
        control.markers.push_back(box_marker);

        return control;
    }

} // namespace b_manipulated_rviz_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(b_manipulated_rviz_ui::MTRvizUI, rviz_common::Panel)
