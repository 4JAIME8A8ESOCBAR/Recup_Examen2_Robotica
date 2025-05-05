#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  //COLISION////////////////////////////////////////////////
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // Cubo 1
  moveit_msgs::msg::CollisionObject cube1;
  cube1.header.frame_id = move_group_interface.getPlanningFrame();
  cube1.id = "cubo1";

  shape_msgs::msg::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions = {1.0, 1.0, 1.0};  // x, y, z

  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = 0.7;
  pose1.position.y = 0.3;
  pose1.position.z = 0.2;

  cube1.primitives.push_back(primitive1);
  cube1.primitive_poses.push_back(pose1);
  cube1.operation = cube1.ADD;
  collision_objects.push_back(cube1);

  // Cubo 2
  moveit_msgs::msg::CollisionObject cube2;
  cube2.header.frame_id = move_group_interface.getPlanningFrame();
  cube2.id = "cubo2";

  shape_msgs::msg::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions = {1.0, 1.0, 1.0};

  geometry_msgs::msg::Pose pose2;
  pose2.orientation.w = 1.0;
  pose2.position.x = 0.0;
  pose2.position.y = -0.7;
  pose2.position.z = 0.2;

  cube2.primitives.push_back(primitive2);
  cube2.primitive_poses.push_back(pose2);
  cube2.operation = cube2.ADD;
  collision_objects.push_back(cube2);
  // Añadir al entorno
  planning_scene_interface.applyCollisionObjects(collision_objects);
  rclcpp::sleep_for(std::chrono::seconds(1));  // Esperar a que se procese
  //////////////////////////////////////////////////////////////////////
  
  int option = 0;
  float orient_posit[4];  // w, x, y, z

  while (rclcpp::ok() && option != 5)
  {
    std::cout << "\n===== MENÚ DE CONTROL DEL ROBOT CINEMATICA INDIRECTA=====" << std::endl;
    std::cout << "1. Ir a posición 1" << std::endl;
    std::cout << "2. Ir a posición 2" << std::endl;
    std::cout << "3. Ir a posición 3" << std::endl;
    std::cout << "4. Coordenadas deseadas por usuario" << std::endl;
    std::cout << "5. Salir" << std::endl;
    std::cout << "Seleccione una opción: ";
    std::cin >> option;

    geometry_msgs::msg::Pose target_pose;

    if (option == 1)
    {
      target_pose.orientation.w = 0.5;
      target_pose.position.x = -0.58;
      target_pose.position.y = 0.2;
      target_pose.position.z = 0.5;
    }
    else if (option == 2)
    {
      target_pose.orientation.w = 0.3;
      target_pose.position.x = 0.9;
      target_pose.position.y = 0.4;
      target_pose.position.z = 0.1;
    }
    else if (option == 3)
    {
      target_pose.orientation.w = 0.0;
      target_pose.position.x = 0.1;
      target_pose.position.y = 0.2;
      target_pose.position.z = 0.9;
    }
    else if (option == 4)
    {
      std::cout << "Ingrese orientación w: "; std::cin >> orient_posit[0];
      std::cout << "Ingrese posición x: "; std::cin >> orient_posit[1];
      std::cout << "Ingrese posición y: "; std::cin >> orient_posit[2];
      std::cout << "Ingrese posición z: "; std::cin >> orient_posit[3];

      target_pose.orientation.w = orient_posit[0];
      target_pose.position.x = orient_posit[1];
      target_pose.position.y = orient_posit[2];
      target_pose.position.z = orient_posit[3];
    }
    else if (option == 5)
    {
      std::cout << "Saliendo del programa..." << std::endl;
      break;
    }
    else
    {
      std::cout << "Opción no válida. Intente nuevamente." << std::endl;
      continue;
    }

    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      bool ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "¡La planificación ha fallado!");
    }
  }

  rclcpp::shutdown();
  return 0;
}
