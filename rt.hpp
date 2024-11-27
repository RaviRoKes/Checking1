#ifndef B_MANIPULATED_RVIZ_UI_MT_RVIZ_UI_HPP
#define B_MANIPULATED_RVIZ_UI_MT_RVIZ_UI_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLineEdit>

namespace b_manipulated_rviz_ui
{

class MTRvizUI : public rviz_common::Panel
{
    Q_OBJECT

public:
    MTRvizUI(QWidget *parent = nullptr);
    ~MTRvizUI() override;

private:
    // Method to create interactive markers
    void createInteractiveMarkers();
    
    // Method to toggle the visibility of a cylinder
    void toggleCylinderVisibility(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    
    // Method to add cylinder to the grid
    void addCylinderToGrid(float radius, float height);

    // Declare the new function to create interactive controls
    visualization_msgs::msg::InteractiveMarkerControl createInteractiveControl(
        const visualization_msgs::msg::InteractiveMarker& marker,
        const visualization_msgs::msg::Marker& box_marker
    );

    // UI elements
    QLineEdit *radius_input_;
    QLineEdit *height_input_;
    QPushButton *add_cylinder_button_;

    // Interactive Markers
    interactive_markers::InteractiveMarkerServer *server_;
    std::vector<visualization_msgs::msg::InteractiveMarker> interactive_markers_;
};

} // namespace b_manipulated_rviz_ui

#endif // B_MANIPULATED_RVIZ_UI_MT_RVIZ_UI_HPP
