# ifndef Optimize_H
# define Optimize_H
# include <iostream>
# include <eigen3/Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
// using Eigen::VectorXd;
class Optimize {
private:
    // MatrixXd points; 
    int N;
    int q_size; 
    double dt = 0.1;
    double m;
    double dT;
    MatrixXd q;
        
public: 

    MatrixXd trajoptimize(MatrixXd points)
    {

        q_size = points.cols()-1; // 총 분할 구간 수
        q = points;
        m = 10.0; //구간당 분할 수
        dT = m*dt; //구간당 소요시간
        N =  q_size * m;

        MatrixXd q_V = MatrixXd(3,q_size+1);
        MatrixXd q_A = MatrixXd(3,q_size+1);

        MatrixXd delta_x = MatrixXd(3,q_size);
        MatrixXd delta_y = MatrixXd(3,q_size);
        MatrixXd delta_z = MatrixXd(3,q_size);
        MatrixXd coeff_x = MatrixXd(3,q_size+1);
        MatrixXd coeff_y = MatrixXd(3,q_size+1);
        MatrixXd coeff_z = MatrixXd(3,q_size+1);

        MatrixXd s_t = MatrixXd(3,N+1);
        MatrixXd v_t = MatrixXd(3,N+1);
        MatrixXd a_t = MatrixXd(3,N+1);
        MatrixXd a_tf = MatrixXd(3,N+1);
        MatrixXd trajtot = MatrixXd(9,N+1);
        int size = 20;
        s_t.col(0) = q.col(0); //set start point
        
        for (int i = 0; i < q_size+1 ; i++){
        
        if (i == 0 )
        {
            q_V.col(i) << (q.col(i+1)-q.col(i))/dT;

        }
        else if (i == q_size)
        {
            q_V.col(i) << (q.col(i)-q.col(i-1))/dT;
        } 
        else
        {
            q_V.col(i) << (q.col(i+1)-q.col(i-1))/dT/2.0;
        }

        }

        for (int i = 0; i < q_size+1 ; i++){
        
        if (i == 0 )
        {      
            q_A.col(i) << (q_V.col(i+1)-q_V.col(i))/dT;
        }
        else if (i == q_size)
        {
            q_A.col(i) << (q_V.col(i)-q_V.col(i-1))/dT;

        }
        else
        {
            q_A.col(i) << (q_V.col(i+1)-q_V.col(i-1))/dT/2.0;
        }
        }

    for (int i = 0; i < q_size ;i++){
        delta_x.col(i) << q(0,i+1)-q(0,i)-q_V(0,i)*dT-(1/2)*q_A(0,i)*pow(dT,2), q_V(0,i+1)-q_V(0,i)-q_A(0,i)*dT, q_A(0,i+1)-q_A(0,i);
        delta_y.col(i) << q(1,i+1)-q(1,i)-q_V(1,i)*dT-(1/2)*q_A(1,i)*pow(dT,2), q_V(1,i+1)-q_V(1,i)-q_A(1,i)*dT, q_A(1,i+1)-q_A(1,i);
        delta_z.col(i) << q(2,i+1)-q(2,i)-q_V(2,i)*dT-(1/2)*q_A(2,i)*pow(dT,2), q_V(2,i+1)-q_V(2,i)-q_A(2,i)*dT, q_A(2,i+1)-q_A(2,i);
        MatrixXd Temp = MatrixXd(3,3);
        Temp << 720, -360*dT, 60*pow(dT,2), -360*dT, 168*pow(dT,2), -24*pow(dT,3), 60*pow(dT,2), -24*pow(dT,3), 3*pow(dT,4); 
        coeff_x.col(i) = (1/pow(dT,5))*Temp*delta_x.col(i);
        coeff_y.col(i) = (1/pow(dT,5))*Temp*delta_y.col(i);
        coeff_z.col(i) = (1/pow(dT,5))*Temp*delta_z.col(i);


    }

    for (int i = 0 ; i <= N ; ++i ){
        float t = (i % int(m)) * dt;
        int j;
        j = i/m ;
        
        s_t(0,i) = pow(t,5)*coeff_x(0,j)/120.0 + pow(t,4)*coeff_x(1,j)/24.0 + pow(t,3)*coeff_x(2,j)/6.0 + (1/2)*q_A(0,j)*pow(t,2) + q_V(0,j)*t + q(0,j);
        s_t(1,i) = pow(t,5)*coeff_y(0,j)/120.0 + pow(t,4)*coeff_y(1,j)/24.0 + pow(t,3)*coeff_y(2,j)/6.0 + (1/2)*q_A(1,j)*pow(t,2) + q_V(1,j)*t + q(1,j);
        s_t(2,i) = pow(t,5)*coeff_z(0,j)/120.0 + pow(t,4)*coeff_z(1,j)/24.0 + pow(t,3)*coeff_z(2,j)/6.0 + (1/2)*q_A(2,j)*pow(t,2) + q_V(2,j)*t + q(2,j); 

        v_t(0,i) = pow(t,4)*coeff_x(0,j)/24.0 + pow(t,3)*coeff_x(1,j)/6.0 + pow(t,2)*coeff_x(2,j)/2.0 + q_A(0,j)*t + q_V(0,j);  
        v_t(1,i) = pow(t,4)*coeff_y(0,j)/24.0 + pow(t,3)*coeff_y(1,j)/6.0 + pow(t,2)*coeff_y(2,j)/2.0 + q_A(1,j)*t + q_V(1,j);
        v_t(2,i) = pow(t,4)*coeff_z(0,j)/24.0 + pow(t,3)*coeff_z(1,j)/6.0 + pow(t,2)*coeff_z(2,j)/2.0 + q_A(2,j)*t + q_V(2,j);
        
        a_t(0,i) = pow(t,3)*coeff_x(0,j)/6.0 + pow(t,2)*coeff_x(1,j)/2.0 + t*coeff_x(2,j) + q_A(0,j);
        a_t(1,i) = pow(t,3)*coeff_y(0,j)/6.0 + pow(t,2)*coeff_y(1,j)/2.0 + t*coeff_y(2,j) + q_A(1,j);
        a_t(2,i) = pow(t,3)*coeff_z(0,j)/6.0 + pow(t,2)*coeff_z(1,j)/2.0 + t*coeff_z(2,j) + q_A(2,j);
    }

    for (int i = size/2+1 ; i < N+1 - size/2 ; i++)  //running average(스무딩)
    {
        for (int j =0 ; j < size  ; j++)
        {
            a_tf(0,i) = a_tf(0,i) + a_t(0,i-size/2.0 + j );
            a_tf(1,i) = a_tf(1,i) + a_t(1,i-size/2.0 + j );
            a_tf(2,i) = a_tf(2,i) + a_t(2,i-size/2.0 + j );        
        }
        a_tf(0,i) = a_tf(0,i)/(size+1);
        a_tf(1,i) = a_tf(1,i)/(size+1);
        a_tf(2,i) = a_tf(2,i)/(size+1);
    }   
    for (int i=0; i<3; i++){
        trajtot.row(i) = s_t.row(i);
    }
    for (int i=0; i<3; i++){
        trajtot.row(i+3) = v_t.row(i);
    }
    for (int i=0; i<3; i++){
        trajtot.row(i+6) = a_tf.row(i);
    }
    
    return trajtot;
    }
};


     




#endif
