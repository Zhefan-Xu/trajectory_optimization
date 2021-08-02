/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
 *    \file   examples/simulation_environment/simple_mpc.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{



 //    // INTRODUCE THE VARIABLES:
 //    // -------------------------
	// // DifferentialState xBody;
	// // DifferentialState xWheel;
	// // DifferentialState vBody;
	// // DifferentialState vWheel;

	// DifferentialState x;
	// DifferentialState y;
	// DifferentialState z;
	// DifferentialState vx;
	// DifferentialState vy;
	// DifferentialState vz;
	// DifferentialState roll;
	// DifferentialState pitch;
	// DifferentialState yaw;

	// Control T;
	// Control roll_d;
	// Control pitch_d;
	// Control yawdot_d;

	// // Disturbance roadExcitation;

	// // Control dampingForce;

	// // double mBody   = 350.0;
	// // double mWheel  = 50.0;
	// // double kSpring = 20000.0;
	// // double kTire   = 200000.0;


 //    // DEFINE A DIFFERENTIAL EQUATION:
 //    // -------------------------------
 //    // DifferentialEquation f;

	// // f << dot(xBody)  == vBody;
	// // f << dot(xWheel) == vWheel;
	// // f << dot(vBody)  == ( -kSpring*xBody + kSpring*xWheel + dampingForce ) / mBody;
	// // f << dot(vWheel) == ( -kTire*xBody - (kTire+kSpring)*xWheel + kTire*roadExcitation - dampingForce ) / mWheel;

	// DifferentialEquation f;
	// f << dot(x) == vx;
	// f << dot(y) == vy;
	// f << dot(z) == vz;
	// f << dot(vx) == T*cos(roll)*sin(pitch)*cos(yaw) + T*sin(roll)*sin(yaw); 
	// f << dot(vy) == T*cos(roll)*sin(pitch)*sin(yaw) - T*sin(roll)*cos(yaw);
	// f << dot(vz) == T*cos(roll)*cos(pitch) - 9.8;
	// f << dot(roll) == (1.0/1) * (1 * roll_d - roll);
	// f << dot(pitch) == (1.0/1) * (1 * pitch_d - pitch);
	// f << dot(yaw) == yawdot_d;


 //    // DEFINE LEAST SQUARE FUNCTION:
 //    // -----------------------------
 // //    Function h;

 // //    h << xBody;
 // //    h << xWheel;
	// // h << vBody;
 // //    h << vWheel;

	// Function h;
	// h << x;
	// h << y;
	// h << z;
	// h << yaw;

 //    DMatrix S(4,4);
 //    DVector r(4);

 //    S.setIdentity();
	// S(0,0) = 10.0;
	// S(1,1) = 10.0;

 //    r.setAll( 0.0 );


 //    // DEFINE AN OPTIMAL CONTROL PROBLEM:
 //    // ----------------------------------
 //    const double t_start = 0.0;
 //    const double t_end   = 1.0;

 //    OCP ocp( t_start, t_end, 20 );

 //    ocp.minimizeLSQ( S, h, r );
	// //ocp.minimizeLagrangeTerm( 0.5*(10.0*xBody*xBody + 10.0*xWheel*xWheel + vBody*vBody + vWheel*vWheel ) );

	// ocp.subjectTo( f );
	// ocp.subjectTo( AT_START, x  == 1 );
	// ocp.subjectTo( AT_START, y == 1 );
	// ocp.subjectTo( AT_START, z  == 1 );
	// ocp.subjectTo( AT_START, vx == 0.0 );
	// ocp.subjectTo( AT_START, vy == 0.0 );
	// ocp.subjectTo( AT_START, vz == 0.0 );

	// ocp.subjectTo( AT_START, roll == 0.0 );
	// ocp.subjectTo( AT_START, pitch == 0.0 );
	// ocp.subjectTo( AT_START, yaw == 0.0 );

	// ocp.subjectTo( 0 <= T <= 2 ); 
	// ocp.subjectTo( -0.5 <= roll_d <= 0.5);
	// ocp.subjectTo( -0.5 <= pitch_d <= 0.5);
	// ocp.subjectTo( -0.5 <= yawdot_d <= 0.5);

	// // ocp.subjectTo( AT_START, xBody  == 0.01 );
	// // ocp.subjectTo( AT_START, xWheel == 0.0 );
	// // ocp.subjectTo( AT_START, vBody  == 0.0 );
	// // ocp.subjectTo( AT_START, vWheel == 0.0 );

	// // ocp.subjectTo( -500.0 <= dampingForce <= 500.0 );
	// // ocp.subjectTo( roadExcitation == 0.0 );


 //    // Additionally, flush a plotting object
 // //    GnuplotWindow window1;//( PLOT_AT_EACH_ITERATION );
	// // window1.addSubplot( xBody, "Body Position [m]" );
	// // window1.addSubplot( xWheel,"Wheel Position [m]" );
	// // window1.addSubplot( vBody, "Body Velocity [m/s]" );
	// // window1.addSubplot( vWheel,"Wheel Velocity [m/s]" );

	// // window1.addSubplot( dampingForce,"Damping Force [N]" );
	// // window1.addSubplot( roadExcitation,"Road Excitation [m]" );


 //    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
 //    // ---------------------------------------------------
 //    OptimizationAlgorithm algorithm(ocp);

 //    // algorithm << window1;

 // // algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
 // //  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	// // algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );

	// //algorithm.set( INTEGRATOR_TOLERANCE, 1e-8 );
	// // algorithm.set( KKT_TOLERANCE, 1e-6 );
	// //algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
	// //algorithm.set( MAX_NUM_ITERATIONS, 1 );

	// algorithm.solve();
	// 	VariablesGrid xd;
	// algorithm.getDifferentialStates(xd); // get solutions

	// cout << xd << endl;

	DifferentialState x;
	DifferentialState y;
	DMatrix A(2,2);
	// A(0, 0) = x; A(1, 0) = 1; A(1, 1) = y;
	cout << A << endl;
}


