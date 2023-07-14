#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

class InteractiveMarkerPub : public rclcpp::Node
{
    public:
        InteractiveMarkerPub() : Node("interactive_marker_pub_node") {
            //Runs the interactive marker server
            server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "interactive_marker_server", this->get_node_base_interface(),
            this->get_node_clock_interface(), this->get_node_logging_interface(),
            this->get_node_topics_interface(), this->get_node_services_interface());

            //Listens for user interaction with the cube
            feedback_sub = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
                "interactive_marker_server/feedback", 10,
                std::bind(&InteractiveMarkerPub::markerMoveCallback, this, std::placeholders::_1));

            //initialize selection cube
            if (firstRender){
                firstRender = false;    
                //Create interactive marker pointer
                auto int_marker = std::make_shared<visualization_msgs::msg::InteractiveMarker>();
                int_marker->header.frame_id = "world";
                int_marker->header.stamp = this->now();
                int_marker->name = "my_marker";
                int_marker->description = "SELECTION MARKER";
                int_marker->scale = 1.0;

                //create interactive marker control
                visualization_msgs::msg::InteractiveMarkerControl control;
                control.always_visible = true;
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
                control.name = "move_3d";
                
                control.markers.push_back(marker_cube(*int_marker));
                int_marker->controls.push_back(control);
                RCLCPP_INFO(this->get_logger(), "Publishing interactive marker");

                server->insert(*int_marker);
                server->applyChanges();
            }
    };
    private:
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
        rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_sub;
        int markerID = 0;
        bool firstRender = true;

        /*
            Marker Cube Creation Function
            Creates a cube marker with the same pose as the interactive marker
            @param int_marker: interactive marker to be used as reference for the cube
            @return marker: cube marker
        */
        visualization_msgs::msg::Marker marker_cube(visualization_msgs::msg::InteractiveMarker int_marker) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.id = markerID++;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.pose = int_marker.pose;

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.type = visualization_msgs::msg::Marker::CUBE;
            
            return marker;
        }

        /*
            Move Marker Callback
            Callback function for when the user moves the marker
            @param feedback: feedback from the interactive marker
        */
        void markerMoveCallback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Marker moved");
            if (feedback->marker_name == "my_marker"){

                auto int_marker = std::make_shared<visualization_msgs::msg::InteractiveMarker>();
                int_marker->header.frame_id = "world";
                int_marker->header.stamp = this->now();
                int_marker->name = "my_marker";
                int_marker->description = "MARKER";
                int_marker->scale = 1.0;
                int_marker->pose = feedback->pose;

                visualization_msgs::msg::InteractiveMarkerControl control;
                control.always_visible = true;
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
                control.name = "move_3d";
                
                control.markers.push_back(marker_cube(*int_marker));
                int_marker->controls.push_back(control);

                server->insert(*int_marker);
                server->applyChanges();
            }
        }

}; //END OF INTERACTIVEMARKERPUB CLASS//

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InteractiveMarkerPub>());
    rclcpp::shutdown();
    return 0;
}
