#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <vector>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_control_node");
  auto logger = rclcpp::get_logger("joint_control_node");

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  int option = 0;
  float joints_d[7];  // j1, j2, j3, j4, j5 ,j6 ,j7

  while (rclcpp::ok() && option != 4)
  {
    std::cout << "\n===== MENÚ DE CONTROL DEL ROBOT =====" << std::endl;
    std::cout << "1. Ir a posición 1" << std::endl;
    std::cout << "2. Ir a posición 2" << std::endl;
    std::cout << "3. Coordenadas deseadas por usuario" << std::endl;
    std::cout << "4. Salir" << std::endl;
    std::cout << "Seleccione una opción: ";
    std::cin >> option;

    std::vector<double> joint_values;

    if (option == 1)
    {
      joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 3.14, 0.78};
    }
    else if (option == 2)
    {
      joint_values = {0.0, 1.57, 0.0, 0.0, 0.0, 3.14, 0.78};
    }
    else if (option == 3)
    {
      std::cout << "Ingrese joint 1: "; std::cin >> joints_d[0];
      std::cout << "Ingrese joint 2: "; std::cin >> joints_d[1];
      std::cout << "Ingrese joint 3: "; std::cin >> joints_d[2];
      std::cout << "Ingrese joint 4: "; std::cin >> joints_d[3];
      std::cout << "Ingrese joint 5: "; std::cin >> joints_d[4];
      std::cout << "Ingrese joint 6: "; std::cin >> joints_d[5];
      std::cout << "Ingrese joint 7: "; std::cin >> joints_d[6];

      joint_values = {joints_d[0], joints_d[1], joints_d[2], joints_d[3], joints_d[4], joints_d[5], joints_d[6]};
    }
    else if (option == 4)
    {
      std::cout << "Saliendo del programa..." << std::endl;
      break;
    }
    else
    {
      std::cout << "Opción no válida. Intente nuevamente." << std::endl;
      continue;
    }


    // Planificación hacia la posición articular
    move_group.setJointValueTarget(joint_values);

    auto const [success, plan] = [&move_group] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      bool ok = static_cast<bool>(move_group.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      RCLCPP_INFO(logger, "¡Planificación exitosa! Ejecutando...");
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "¡Falló la planificación!");
    }
  }

  rclcpp::shutdown();
  return 0;
}
