#include "../include/dtvr_message_packing_unpacking/dtvr_message_packing_unpacking.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dtvr_message_packing_unpacking_node");
  dtvr_message_packing_unpacking node;
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin
}
