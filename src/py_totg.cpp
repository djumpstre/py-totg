

#include <iostream>
#include <cstdio>
#include <vector>
#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/LU>
#include "Trajectory.h"
#include "Path.h"

using namespace std;
using namespace Eigen;
namespace py = pybind11;


class ToTg
{
    public:
        ToTg(const int dof_, const double time_step, const VectorXd &vel_m, const VectorXd &acc_m) :
            dof(dof_), timeStep(time_step), vel_max(vel_m), acc_max(acc_m) {};

        ~ToTg() {};

        VectorXd getMaxVelocity() {return vel_max;};
        VectorXd getMaxAcceleration() {return acc_max;};

        void setWayPoints(std::vector<std::vector<double>> &points, double max_deviation);

        void computeTrajectory();

        std::vector<std::vector<double>> getResPosition();
        std::vector<std::vector<double>> getResVelocity();

    private:
        int dof;
        double timeStep;
        double maxDeviation = 0;
        VectorXd vel_max;
        VectorXd acc_max;
        list<VectorXd> waypoints;

        std::vector<std::vector<double>> resPosition;
        std::vector<std::vector<double>> resVelocity;
};


void ToTg::setWayPoints(std::vector<std::vector<double>> &points, double max_deviation)
{
    maxDeviation = max_deviation;
    VectorXd waypoint(dof);
    waypoints.clear();

    // printf("Got %d Waypoints\n", points.size());

    for (int i=0; i < points.size(); i++)
        {
             //Map<VectorXd> waypoint(&points[i][0], dof);
            waypoint = VectorXd(dof);
            for (int j=0; j<dof; j++){
                waypoint[j] = (double) points[i][j];
            }
            // printf("%d : %7.4f %7.4f \n", i, points[i][0], points[i][1] );
            waypoints.push_back(waypoint);
        }
}


void ToTg::computeTrajectory()
{
    Path path(waypoints, maxDeviation);
    Trajectory trajectory(path, vel_max, acc_max, timeStep);
    resPosition.clear();
    resVelocity.clear();
    // printf("Number of waypoints used in ToTg:  %d \n", path.getLength());
	// trajectory.outputPhasePlaneTrajectory();
	if(trajectory.isValid()) {
		double duration = trajectory.getDuration();
        // printf("Trajectory duration:  %f \n", duration);
		std::vector<double> row;
		row.resize(dof);
		for(double t = 0.0; t < duration; t += timeStep) {
		    VectorXd::Map(&row[0], dof) = trajectory.getPosition(t);
		    resPosition.push_back(row);
		    VectorXd::Map(&row[0], dof) = trajectory.getVelocity(t);
		    resVelocity.push_back(row);
		};
        VectorXd::Map(&row[0], dof) = trajectory.getPosition(duration);
		    resPosition.push_back(row);
		    VectorXd::Map(&row[0], dof) = trajectory.getVelocity(duration);
		    resVelocity.push_back(row);
//		cout << "Trajectory duration: " << duration << " s" << endl << endl;
//		cout << "Time      Position                  Velocity" << endl;
//		for(double t = 0.0; t < duration; t += 0.1) {
//			printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t,
//			       trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
//				   trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
//		}
//		printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
//			trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
	}
	else {

		cout << "Trajectory generation failed." << endl;
	}
    // printf("Finish.\n");
}


std::vector<std::vector<double>> ToTg::getResPosition(){
    return resPosition;
}

std::vector<std::vector<double>> ToTg::getResVelocity(){
    return resVelocity;
}


PYBIND11_MODULE(py_totg, m)
{
    // Description of the module
    m.doc() = R"pbdoc(
        Pybind11 Wrapper of Time Optimal Trajectory Generation Algorithmus
        -----------------------
        .. currentmodule:: py_totg
    )pbdoc";

    // Binding code
    py::class_<ToTg>(m, "ToTg")
        .def(py::init<const int, double, const VectorXd &, const VectorXd &>())
        .def("get_max_velocity", &ToTg::getMaxVelocity)
        .def("get_max_acceleration", &ToTg::getMaxAcceleration)
        .def("set_waypoints", &ToTg::setWayPoints)
        .def("compute_trajectory", &ToTg::computeTrajectory)
        .def("get_res_position", &ToTg::getResPosition)
        .def("get_res_velocity", &ToTg::getResVelocity)
        ;

    // Version_INFO
    #ifdef VERSION_INFO
        m.attr("__version__") = VERSION_INFO;
    #else
        m.attr("__version__") = "dev";
    #endif

}
