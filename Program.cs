// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using com.robotraconteur.robotics.robot;
using Mono.Options;
using RobotRaconteur;
using RobotRaconteur.Companion.InfoParser;
using RobotRaconteur.Companion.Util;

namespace GazeboModelRobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)
        {

            bool shouldShowHelp = false;
            string robot_info_file = null;
            string robot_name = null;
            ushort tcp_port = 58653;
            string node_name = "gazebo_robot";
            string model_name = null;
            string gazebo_url = null;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "robot-name=", "override the robot device name", n=>robot_name = n },
                { "model-name=", "the gazebo model to control", n=> model_name=n },
                { "gazebo-url=", "url for the Robot Raconteur Gazebo plugin", n=> gazebo_url = n },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null }
            };

            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("GazeboModelRobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `GazeboModelRobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: GazeboModelRobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                return 0;
            }

            if (robot_info_file == null)
            {
                Console.WriteLine("error: robot-info-file must be specified");
                return 1;
            }

            if (model_name == null)
            {
                Console.WriteLine("error: model-name must be specified");
                return 1;
            }

            if (gazebo_url == null)
            {
                Console.WriteLine("error: gazebo_url must be specified");
                return 1;
            }

            var robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file,robot_name);
            robot_info.Item1.robot_capabilities &= (uint)(RobotCapabilities.jog_command | RobotCapabilities.position_command | RobotCapabilities.trajectory_command);

            RobotOperationalMode robot_op_mode = RobotOperationalMode.auto;
            var robot_class = robot_info.Item1?.device_info?.device_classes?.Find(x => x?.class_identifier?.name == "robot");
            if (robot_class?.subclasses != null)
            {
                if (robot_class.subclasses.Contains("cobot"))
                {
                    robot_op_mode = RobotOperationalMode.cobot;
                }
            }


            using (robot_info.Item2)
            {                                
     
                using (var node_setup = new ServerNodeSetup(node_name, tcp_port,args))
                {
                    using (var robot = new GazeboRobot(robot_info.Item1, gazebo_url, model_name, robot_op_mode))
                    {
                        robot._start_robot();
                        var robot_service_ctx = RobotRaconteurNode.s.RegisterService("robot", "com.robotraconteur.robotics.robot", robot);
                        robot_service_ctx.SetServiceAttributes(AttributesUtil.GetDefaultServiceAtributesFromDeviceInfo(robot_info.Item1.device_info));

                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();

                    }
                }
                
            }

            return 0;

        }
    }
}
