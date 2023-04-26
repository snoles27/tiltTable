#include<Eigen/Dense>

Eigen::Vector2f nrstep(Eigen::Vector2f x, Eigen::Vector2f (*f)(Eigen::Vector2f), Eigen::Matrix2f (*fdot)(Eigen::Vector2f));
Eigen::Vector2f newtownRaphson(Eigen::Vector2f x0, Eigen::Vector2f (*f)(Eigen::Vector2f), Eigen::Matrix2f (*fdot)(Eigen::Vector2f));