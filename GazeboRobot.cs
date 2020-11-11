using com.robotraconteur.robotics.robot;
using RobotRaconteur;
using RobotRaconteur.Companion.Robot;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Rox = GeneralRoboticsToolbox;

namespace GazeboModelRobotRaconteurDriver
{
    public class GazeboRobot : AbstractRobot, IDisposable
    {

        protected internal string gazebo_url;
        protected internal string gazebo_model_name;
        protected internal experimental.gazebo.Server gazebo_server;
        protected internal experimental.gazebo.Model gazebo_robot;
        protected internal experimental.gazebo.JointController gazebo_controller;
        protected internal Wire<Dictionary<string, double>>.WireConnection gazebo_controller_position;
        protected internal Wire<Dictionary<string,double>>.WireConnection gazebo_controller_position_command;
        protected internal Wire<Dictionary<string, double>>.WireConnection gazebo_controller_velocity;
        protected internal Wire<Dictionary<string, double>>.WireConnection gazebo_controller_velocity_command;
        protected internal bool connecting = false;
        protected internal bool connected = false;
        protected internal RobotOperationalMode gazebo_operational_mode;

        protected internal GeneralRoboticsToolbox.Robot[] rox_robots_no_limits;

        public GazeboRobot(RobotInfo robot_info, string gazebo_url, string gazebo_model_name, RobotOperationalMode gazebo_operational_mode) : base(robot_info, 0)
        {
            this.gazebo_url = gazebo_url;
            this.gazebo_model_name = gazebo_model_name;
            this.gazebo_operational_mode = gazebo_operational_mode;

            this.rox_robots_no_limits = new Rox.Robot[robot_info.chains.Count];
            for (int i = 0; i < robot_info.chains.Count; i++)
            {
                this.rox_robots_no_limits[i] = RobotInfoConverter.ToToolboxRobot(robot_info, i);
                rox_robots_no_limits[i].Joint_lower_limit = null;
                rox_robots_no_limits[i].Joint_upper_limit = null;
                rox_robots_no_limits[i].Joint_vel_limit = null;
                rox_robots_no_limits[i].Joint_acc_limit = null;
            }
        }

        protected override bool _verify_communication(long now)
        {            
            lock(this)
            {
                if (!connected)
                {
                    _last_robot_state = 0;
                }
                else
                {
                    _last_robot_state = now;
                    _ready = true;
                    _enabled = true;
                    _homed = true;
                    _error = false;
                    _operational_mode = gazebo_operational_mode;
                }

                if (!connected && !connecting)
                {
                    Task.Run(_connect_gazebo);
                }
            }



            return base._verify_communication(now);
        }

        protected virtual void _gazebo_client_listener(ServiceStub context, ClientServiceListenerEventType ev, object parameter)
        {
            if (ev == ClientServiceListenerEventType.ClientClosed || ev == ClientServiceListenerEventType.ServicePathReleased)
            {
                lock (this)
                {
                    if (ev == ClientServiceListenerEventType.ServicePathReleased)
                    {
                        if (((string)parameter) != model_rr_path)
                        {
                            return;
                        }
                    }

                    connected = false;
                    gazebo_server = null;
                    gazebo_robot = null;
                    gazebo_controller = null;
                    gazebo_controller_position = null;
                    gazebo_controller_position_command = null;
                    gazebo_controller_velocity = null;
                    gazebo_controller_velocity_command = null;
                }

                Console.WriteLine("Connection to gazebo lost");
            }
        }

        protected string model_rr_path;
        protected virtual async Task _connect_gazebo()
        {
            try
            {
                experimental.gazebo.Server server;
                experimental.gazebo.Model model;
                experimental.gazebo.JointController controller;
                Wire<Dictionary<string, double>>.WireConnection controller_position;
                Wire<Dictionary<string, double>>.WireConnection controller_position_command;
                Wire<Dictionary<string, double>>.WireConnection controller_velocity;
                Wire<Dictionary<string, double>>.WireConnection controller_velocity_command;

                experimental.gazebo.Server old_server;

                lock (this)
                {
                    if (connected || connecting)
                    {
                        return;
                    }
                    connecting = true;
                    connected = false;
                    old_server = gazebo_server;
                    gazebo_server = null;
                    gazebo_robot = null;
                    gazebo_controller = null;
                    gazebo_controller_position = null;
                    gazebo_controller_position_command = null;
                    gazebo_controller_velocity = null;
                    gazebo_controller_velocity_command = null;
                }

                try
                {
                    if (old_server != null)
                    {
                        RobotRaconteurNode.s.AsyncDisconnectService(old_server).ContinueWith(c => { var ignored = c.Exception; },
                                    TaskContinuationOptions.OnlyOnFaulted |
                                    TaskContinuationOptions.ExecuteSynchronously);
                    }
                }
                catch (Exception) { }

                Console.WriteLine($"Begin connect to gazebo with url {gazebo_url} and model {gazebo_model_name}");
                server = (experimental.gazebo.Server)await RobotRaconteurNode.s.AsyncConnectService(gazebo_url, null, null, _gazebo_client_listener, null);
                var a_server = (experimental.gazebo.async_Server)server;
                var w_names = await a_server.async_get_world_names();
                var w = await a_server.async_get_worlds(w_names[0]);
                var a_w = (experimental.gazebo.async_World)w;
                model = await a_w.async_get_models(gazebo_model_name);

                model_rr_path = RobotRaconteurNode.s.GetObjectServicePath(model);

                var a_model = (experimental.gazebo.async_Model)model;
                try
                {
                    await a_model.async_destroy_joint_controller();
                }
                catch (Exception) { }
                try
                {
                    await a_model.async_destroy_kinematic_joint_controller();
                }
                catch (Exception) { }

                await a_model.async_create_kinematic_joint_controller();
                controller = await a_model.async_get_kinematic_joint_controller();
                var a_controller = (experimental.gazebo.async_JointController)controller;

                foreach (var joint_name in _joint_names)
                {
                    await a_controller.async_add_joint(joint_name);
                }

                controller_position = await controller.joint_position.AsyncConnect();
                controller_position_command = await controller.joint_position_command.AsyncConnect();
                controller_velocity = await controller.joint_velocity.AsyncConnect();
                controller_velocity_command = await controller.joint_velocity_command.AsyncConnect();

                lock (this)
                {
                    gazebo_server = server;
                    gazebo_robot = model;
                    gazebo_controller = controller;
                    gazebo_controller_position = controller_position;
                    gazebo_controller_position_command = controller_position_command;
                    gazebo_controller_velocity = controller_velocity;
                    gazebo_controller_velocity_command = controller_velocity_command;
                    connected = true;
                    connecting = false;
                }

                controller_position.WireValueChanged += _on_joint_position;
                controller_velocity.WireValueChanged += _on_joint_velocity;

                Console.WriteLine("Connected to gazebo");


            }
            catch (Exception e)
            {
                lock(this)
                {
                    connected = false;
                }
                Console.WriteLine("Error connection to gazebo server: " + e.ToString());
                //TODO: send to event log

                // Backoff to prevent flooding gazebo
                await Task.Delay(500);

                lock (this)
                {
                    gazebo_server = null;
                    gazebo_robot = null;
                    gazebo_controller = null;
                    gazebo_controller_position = null;
                    gazebo_controller_position_command = null;
                    gazebo_controller_velocity = null;
                    gazebo_controller_velocity_command = null;
                    connecting = false;
                }
                
                
                
            }
        }

        public override void Dispose()
        {
            base.Dispose();
        }

        protected override Task _send_disable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_enable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_reset_errors()
        {
            throw new NotImplementedException();
        }

        protected override void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd)
        {
            if (joint_pos_cmd!=null)
            {
                var cmd = new Dictionary<string, double>();
                for(int i=0; i<_joint_names.Length; i++)
                {
                    cmd[_joint_names[i]] = joint_pos_cmd[i];
                }

                gazebo_controller_position_command.OutValue = cmd;
            }
        }

        protected virtual void _on_joint_position(Wire<Dictionary<string,double>>.WireConnection c, Dictionary<string,double> value, TimeSpec ts)
        {
            if (value == null) return;
            var pos = new double[_joint_names.Length];
            for (int i = 0; i < _joint_names.Length; i++)
            {
                pos[i] = value[_joint_names[i]];
            }

            com.robotraconteur.geometry.Pose[] ep_pose = new com.robotraconteur.geometry.Pose[rox_robots_no_limits.Length];
            try
            {
                for (int i = 0; i < rox_robots_no_limits.Length; i++)
                {
                    var joint_numbers = this._robot_info.chains[i].joint_numbers;
                    double[] pos1 = new double[joint_numbers.Length];
                    for (int j=0; j<joint_numbers.Length; j++)
                    {
                        pos1[j] = pos[joint_numbers[j]];
                    }
                    var rox_ep_pose = GeneralRoboticsToolbox.Functions.Fwdkin(rox_robots_no_limits[i], pos1);

                    ep_pose[i] = RobotRaconteur.Companion.Converters.GeometryConverter.ToPose(rox_ep_pose);
                }
            }
            catch (Exception e)
            {
                ep_pose = null;
            }

            lock (this)
            {
                _joint_position = pos;
                _endpoint_pose = ep_pose;
                _last_joint_state = _stopwatch.ElapsedMilliseconds;
                _last_endpoint_state = _stopwatch.ElapsedMilliseconds;
            }
        }

        protected virtual void _on_joint_velocity(Wire<Dictionary<string, double>>.WireConnection c, Dictionary<string,double> value, TimeSpec ts)
        {
            if (value == null) return;
            var vel = new double[_joint_names.Length];
            for (int i = 0; i < _joint_names.Length; i++)
            {
                vel[i] = value[_joint_names[i]];
            }
            com.robotraconteur.geometry.SpatialVelocity[] ep_vel = new com.robotraconteur.geometry.SpatialVelocity[rox_robots_no_limits.Length];
            try
            {
                for (int i = 0; i < _rox_robots.Length; i++)
                {
                    var joint_numbers = this._robot_info.chains[i].joint_numbers;
                    double[] vel1 = new double[joint_numbers.Length];
                    for (int j = 0; j < joint_numbers.Length; j++)
                    {
                        vel1[j] = vel[joint_numbers[j]];
                    }
                    var rox_ep_jac = GeneralRoboticsToolbox.Functions.Robotjacobian(_rox_robots[i], vel1);
                    var rox_ep_vel = rox_ep_jac.Multiply(MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(vel));

                    ep_vel[i] = RobotRaconteur.Companion.Converters.GeometryConverter.ToSpatialVelocity(rox_ep_vel);
                }
            }
            catch (Exception e)
            {
                ep_vel = null;
            }


            lock (this)
            {
                _endpoint_vel = ep_vel;
                _joint_velocity = vel;
            }
        }
    }
}
