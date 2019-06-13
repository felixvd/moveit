/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#include <moveit/handeye_calibration_solver/handeye_solver_default.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_solver_default";

void HandEyeSolverDefault::initialize()
{
  solver_names_ = {"Daniilidis1999", "ParkBryan1994", "TsaiLenz1989"};
  camera_robot_pose_ = Eigen::Isometry3d::Identity();
}

const std::vector<std::string>& HandEyeSolverDefault::getSolverNames()
{
  return solver_names_;
}

const Eigen::Isometry3d& HandEyeSolverDefault::getCameraRobotPose()
{
  return camera_robot_pose_;
}

bool HandEyeSolverDefault::solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world, 
                                 const std::vector<Eigen::Isometry3d>& object_wrt_sensor, 
                                 SENSOR_MOUNT_TYPE setup,
                                 const std::string& solver_name)
{
  // Check the size of the two sets of pose sample equal
  if (effector_wrt_world.size() != object_wrt_sensor.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "The sizes of two input pose samples must be equal");
    return false;
  }

  char program_name[7] = "python";
  Py_SetProgramName(program_name);
  static int run_times{0};
  if (run_times == 0) // Numpy can only be loaded once, otherwise will segfault
  {
    ++run_times;
    Py_Initialize();
    atexit(Py_Finalize);
  } 
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Python C API start");

  // Load numpy c api
  if (_import_array() < 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error importing numpy: ");
    return false;
  }

  PyObject *p_name, *p_module, *p_class, *p_instance, *p_func_add_sample, *p_func_solve; 
  PyObject *p_args, *p_value;

  // Import handeye.calibrator python module
  p_name = PyString_FromString("handeye.calibrator");
  p_module = PyImport_Import(p_name);
  Py_DECREF(p_name);
  if (!p_module)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load python module: " << "handeye.calibrator");
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator class
  p_class = PyObject_GetAttrString(p_module, "HandEyeCalibrator");
  Py_DECREF(p_module);
  if (!p_class || !PyCallable_Check(p_class))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find \"HandEyeCalibrator\" python class");
    PyErr_Print();
    return false;
  }

  // Parse sensor mount type
  if (setup == EYE_TO_HAND)
    p_value = PyString_FromString("Fixed");
  else if (setup == EYE_IN_HAND)
    p_value = PyString_FromString("Moving");
  if (!p_value)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't creat sensor mount type python value");
    Py_DECREF(p_class);
    PyErr_Print();
    return false;
  }

  // Create handeye.calibrator.HandEyeCalibrator instance
  p_args = PyTuple_New(1);
  PyTuple_SetItem(p_args, 0, p_value);
  if (!p_args)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't build python arguments");
    Py_DECREF(p_class);
    PyErr_Print();
    return false;
  } 
  p_instance = PyEval_CallObject(p_class, p_args);
  Py_DECREF(p_args);
  Py_DECREF(p_class);
  if (!p_instance)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create \"HandEyeCalibrator\" python instance");
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator.add_sample method
  p_func_add_sample = PyObject_GetAttrString(p_instance, "add_sample");
  if (!p_func_add_sample || !PyCallable_Check(p_func_add_sample))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'add_sample' method");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  // Add sample poses to handeye.calibrator.HandEyeCalibrator instance
  size_t pose_num = effector_wrt_world.size();
  PyArrayObject *np_arg_eef_wrt_base[pose_num], *np_arg_obj_wrt_sensor[pose_num];
  PyObject *p_array_eef_wrt_base[pose_num], *p_array_obj_wrt_sensor[pose_num];
  PyObject* p_args_sample[pose_num];
  npy_intp dims[2]{ARRAY_SIZE, ARRAY_SIZE};
  const int ND{2};
  // Using C array to store the pyarray data, which will be automatically freed
  double c_arr_eef_wrt_world[pose_num][ARRAY_SIZE][ARRAY_SIZE];
  double c_arr_obj_wrt_sensor[pose_num][ARRAY_SIZE][ARRAY_SIZE];
  for (size_t i = 0; i < pose_num; ++i)
  {
    // Convert effector_wrt_world[i] from Eigen::isometry3d to PyArrayObject
    if (toCArray(effector_wrt_world[i], c_arr_eef_wrt_world[i]))
    {
      p_array_eef_wrt_base[i] = PyArray_SimpleNewFromData(ND, dims, NPY_DOUBLE, (void*)(c_arr_eef_wrt_world[i]));
      if (!p_array_eef_wrt_base[i])
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
        Py_DECREF(p_func_add_sample);
        Py_DECREF(p_instance);
        PyErr_Print();
        return false;
      }
      np_arg_eef_wrt_base[i] = (PyArrayObject*)(p_array_eef_wrt_base[i]);
      if (PyArray_NDIM(np_arg_eef_wrt_base[i]) == 2) // Check PyArrayObject dims are 4x4
      {
        npy_intp * py_array_dims = PyArray_DIMS(np_arg_eef_wrt_base[i]);
        if ( py_array_dims[0] != 4 || py_array_dims[1] != 4)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
          Py_DECREF(np_arg_eef_wrt_base[i]);
          Py_DECREF(p_func_add_sample);
          Py_DECREF(p_instance);
          return false;
        }
      }
    }

    // Convert object_wrt_sensor[i] from Eigen::isometry3d to PyArrayObject
    if (toCArray(object_wrt_sensor[i], c_arr_obj_wrt_sensor[i]))
    {
      p_array_obj_wrt_sensor[i] = PyArray_SimpleNewFromData(ND, dims, NPY_DOUBLE, (void*)(c_arr_obj_wrt_sensor[i]));
      if (!p_array_obj_wrt_sensor[i])
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
        Py_DECREF(p_func_add_sample);
        Py_DECREF(p_instance);
        PyErr_Print();
        return false;
      }
      np_arg_obj_wrt_sensor[i] = (PyArrayObject*)(p_array_obj_wrt_sensor[i]);
      if (PyArray_NDIM(np_arg_obj_wrt_sensor[i]) == 2) // Check PyArrayObject dims are 4x4
      {
        npy_intp * py_array_dims = PyArray_DIMS(np_arg_obj_wrt_sensor[i]);
        if ( py_array_dims[0] != 4 || py_array_dims[1] != 4)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
          Py_DECREF(np_arg_obj_wrt_sensor[i]);
          Py_DECREF(p_func_add_sample);
          Py_DECREF(p_instance);
          return false;
        }
      }
    }

    // Asign sample poses to 'HandEyeCalibrator' instance
    p_args_sample[i] = Py_BuildValue("OO", np_arg_eef_wrt_base[i], np_arg_obj_wrt_sensor[i]);
    if (!p_args_sample[i])
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create argument tuple for 'add_sample' method");
      Py_DECREF(p_func_add_sample);
      Py_DECREF(p_instance);
      PyErr_Print();
      return false;
    }
    p_value = PyEval_CallObject(p_func_add_sample, p_args_sample[i]);
    if (!p_value)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error calling 'add_sample' method");
      Py_DECREF(p_func_add_sample);
      Py_DECREF(p_instance);
      PyErr_Print();
      return false;
    }
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "num_samples: " << PyInt_AsLong(p_value));
    Py_DECREF(p_value);
  }
  Py_DECREF(p_func_add_sample);

  // print the pair of transforms as python arguments
  for (size_t i = 0; i < pose_num; i++)
  {
    std::stringstream ss;
    ss << "\nnp_arg_eef_wrt_base";
    for (size_t m = 0; m < ARRAY_SIZE; m++)
    {
      ss << "\n";
      for (size_t n = 0; n < ARRAY_SIZE; n++)
        ss << *(double *)PyArray_GETPTR2(np_arg_eef_wrt_base[i], m, n) << " ";
    }
    ss << "\nnp_arg_obj_wrt_sensor";
    for (size_t m = 0; m < ARRAY_SIZE; m++)
    {
      ss << "\n";
      for (size_t n = 0; n < ARRAY_SIZE; n++)
        ss << *(double *)PyArray_GETPTR2(np_arg_obj_wrt_sensor[i], m, n) << " ";
    }  
    ROS_DEBUG_STREAM_NAMED(LOGNAME, ss.str());
  }

  // Import handeye.solver python module
  p_name = PyString_FromString("handeye.solver");
  p_module = PyImport_Import(p_name);
  Py_DECREF(p_name);
  if (!p_module)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load python module: " << "handeye.solver");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  // Find handeye.solver.solver_name class
  p_class = PyObject_GetAttrString(p_module, solver_name.c_str());
  Py_DECREF(p_module);
  if (!p_class || !PyCallable_Check(p_class))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find \"" << solver_name << "\" python class");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator.solve method
  p_func_solve = PyObject_GetAttrString(p_instance, "solve");
  if (!p_func_solve || !PyCallable_Check(p_func_solve))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'solve' method");
    Py_DECREF(p_class);
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  // Create argument list for 'solve' method
  p_args = Py_BuildValue("{s:O}", "method", p_class);
  Py_DECREF(p_class);
  if (!p_args)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create argument tuple for 'solve' method");
    Py_DECREF(p_instance);
    PyErr_Print();
    return false;
  }

  // Call 'solve' method to solve AX=XB problem
  p_value = PyEval_CallObjectWithKeywords(p_func_solve, NULL, p_args);
  Py_DECREF(p_args);
  Py_DECREF(p_func_solve);
  for (int i = 0; i < pose_num; ++i) Py_DECREF(p_args_sample[i]);
  Py_DECREF(p_instance);
  if (!p_value)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error calling 'solve' method");
    PyErr_Print();
    return false;
  }
  PyArrayObject* np_ret = (PyArrayObject*)p_value;
  if (!PyArray_Check(p_value) || PyArray_NDIM(np_ret) != 2 || PyArray_NBYTES(np_ret) != sizeof(double)*16)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Did not return a valid array");
    Py_DECREF(p_value);
    PyErr_Print();
    return false; 
  }

  std::stringstream ss;
  ss << "\n Result camera-robot pose";
  for (size_t m = 0; m < ARRAY_SIZE; m++)
  {
    ss << "\n";
    for (size_t n = 0; n < ARRAY_SIZE; n++)
    {
      double item = *(double *)PyArray_GETPTR2(np_ret, m, n);
      camera_robot_pose_(m, n) = item;
      ss << item << " ";
    }
  }
  ROS_DEBUG_STREAM_NAMED(LOGNAME, ss.str());

  Py_DECREF(p_value);
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Python C API end");
  return true;
}  

bool HandEyeSolverDefault::toCArray(const Eigen::Isometry3d& pose, double (* c_arr)[ARRAY_SIZE])
{
  const Eigen::MatrixXd& mat = pose.matrix();

  if (mat.rows() == ARRAY_SIZE && mat.cols() == ARRAY_SIZE)
  {
    for (size_t i = 0; i < ARRAY_SIZE; ++i)
      for (size_t j = 0; j < ARRAY_SIZE; ++j)
        c_arr[i][j] = mat(i, j);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error matrix dims: " << mat.rows() << "x" << mat.cols());
    return false;
  }
}

} // namespace moveit_handeye_calibration
