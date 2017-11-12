// Copyright 2017 Napassorn napalerd@bu.edu
#include <iostream>
#include <string>
#include <algorithm>
#include <utility>
#include <cmath>
#include <complex>
#include <vector>

using std::cout;
using std::string;
using std::pair;
using std::vector;
using std::complex;

class Vehicle {
 public:
  Vehicle();
  Vehicle(string, double, double, double, double, int);

  friend std::ostream& operator<<(std::ostream& out, const Vehicle& car1);
  pair<double, double> diffloc(const Vehicle& other) const;
  pair<double, double> diffvel(const Vehicle& other) const;
  string getID() const;
  bool getpaired() const;
  void setpaired(const bool&);
  int getind() const;

 private:
  string ID;
  pair <double, double> loc;
  pair <double, double> vel;
  int ind;
  bool paired;
};

class Collision {
 public:
  Collision();
  Collision(double, int, int);

  bool operator < (const Collision&) const;
  friend std::ostream& operator<<(std::ostream& out, const Collision& c1);
  pair <int, int> getpairind() const;
  double gettimeCollide() const;

 private:
  int car1;
  int car2;
  double timeCollide;
};

Vehicle::Vehicle() {
  ID = "origin";
  loc = pair <double, double> (0, 0);
  vel = pair <double, double> (0, 0);
  ind = -1;
  paired = false;
}
Vehicle::Vehicle(string s, double rx, double ry, double vx, double vy, int i) {
  ID = s;
  loc = pair <double, double> (rx, ry);
  vel = pair <double, double> (vx, vy);
  ind = i;
  paired = false;  //  alive or not
}
Collision::Collision() {
  timeCollide = -1;
  car1 = 0;
  car2 = 0;
}
Collision::Collision(double t, int a, int b) {
  timeCollide = t;
  car1 = a;
  car2 = b;
}
bool Collision::operator<(const Collision& c) const {
  return timeCollide < c.timeCollide;
}
std::ostream& operator<<(std::ostream& out, const Vehicle& car1) {
  out << car1.ID << ' ' << car1.loc.first << ' ' << car1.loc.second << ' '
      << car1.vel.first << ' ' << car1.vel.second;
  return out;
}
std::ostream& operator<<(std::ostream& out, const Collision& c1) {
  out << c1.timeCollide << ' ' << c1.car1 << ' ' << c1.car2;
  return out;
}
string Vehicle::getID() const {
  return this->ID;
}
int Vehicle::getind() const {
  return this->ind;
}
bool Vehicle::getpaired() const {
  return this->paired;
}
void Vehicle::setpaired(const bool& t) {
  this->paired = t;
}
pair <int, int> Collision::getpairind() const {
  return pair<int, int> (car1, car2);
}
double Collision::gettimeCollide() const {
  return timeCollide;
}  //  Functions
//  Position vector p = r + t*v (r), Calculate delta r and v
pair<double, double> Vehicle::diffloc(const Vehicle& other) const {
  pair<double, double> dr;
  dr.first = other.loc.first - this->loc.first;
  dr.second = other.loc.second - this->loc.second;
  return dr;
}
pair<double, double> Vehicle::diffvel(const Vehicle& other) const {
  pair<double, double> dv;
  dv.first = other.vel.first - this->vel.first;
  dv.second = other.vel.second - this->vel.second;
  return dv;
}  //  calculate abs of delta r and v
double findMag(const pair<double, double> &d) {
  return (d.first*d.first)+(d.second*d.second);
}  //  calculate the coeff in front of t
double middleCoeff(const pair<double, double> &dr,
                   const pair<double, double> &dv) {
  return (2*dr.first*dv.first + 2*dr.second*dv.second);
}
pair<double, double> findTime(const pair<double, double> &dr,
                              const pair<double, double> &dv) {
  const double disMin = 100;
  double A = findMag(dv);
  double B = middleCoeff(dr, dv);
  double C = findMag(dr) - disMin;  //  At^2 +  Bt + C = 0
  double discrim = B*B - A*C*4;
  double t1, t2;
  if (discrim >= 0) {
    t1 = (-B + sqrt(discrim)) / (A*2);
    t2 = (-B - sqrt(discrim)) / (A*2);
    if (t1 < 0) t1 = -1;  //  if negative time
    if (t2 < 0) t2 = -1;  // if negative time
  } else {  //  if complex
    t1 = -1;
    t2 = -1;
  }
  return pair<double, double> (t1, t2);
}
double findCollide(const pair<double, double> &dr,
                   const pair<double, double> &dv) {
  pair<double, double>  time = findTime(dr, dv);
  if (time.first == -1 and time.second == -1) {
    return time.first;
  } else if (time.first == -1 and not time.second == -1) {
    if (time.second > 0) return time.second;
  } else if (not time.first == -1 and time.second == -1) {
    if (time.first > 0) return time.first;
  } else if (time.first < time.second) {
    return time.first;
  } else if (time.first > time.second) {
    return time.second;
  } else if (time.first == time.second) {
    return time.first;
  }
  return -1;
}
int main() {
  string ID;
  int ii{0};
  double r1, r2, v1, v2, timeCollision;
  vector <Vehicle> cars, survivors;
  while (std::cin >> ID >> r1 >> r2 >> v1 >> v2) {
    cars.push_back(Vehicle(ID, r1, r2, v1, v2, ii));
    ++ii;  //  assign index
  }
  int totCars = cars.size();
  pair<double, double> dr, dv;
  vector <Collision> allCollide;
//  saving all possible collisions in vector of class and sort, works
  for (int i{0}; i < totCars; ++i) {
    for (int j{i+1}; j < totCars; ++j) {
      dr = cars[i].diffloc(cars[j]);
      dv = cars[i].diffvel(cars[j]);
      timeCollision = findCollide(dr, dv);
      if (timeCollision != -1) {
        allCollide.push_back(Collision(timeCollision,
                                       cars[i].getind(), cars[j].getind()));
      }
    }
  }
  std::sort(allCollide.begin(), allCollide.end());
  //  move dead cars to new vector and change bool in Vehicle class
  int car1, car2;
  vector<Collision> realCollide;
  double finalTime;
  for (int i{0}; i < allCollide.size(); ++i) {
    pair<int, int> carind = allCollide[i].getpairind();
    finalTime = allCollide[i].gettimeCollide();
    car1 = carind.first;
    car2 = carind.second;
    if (not cars[car1].getpaired() and not cars[car2].getpaired()) {
      cars[car1].setpaired(true);
      cars[car2].setpaired(true);
      realCollide.push_back(Collision(finalTime, car1, car2));
    }
  }  //  Get survivor list
  for (int k{0}; k < totCars; ++k) {
    if (not cars[k].getpaired()) {
      survivors.push_back(cars[k]);
    }
  }  //  Display results
  cout << "there are " << totCars << " vehicles" << '\n';
  cout << "collision report\n";
  if (realCollide.empty()) {
    cout << "none\n";
  } else {
    for (int i{0}; i < realCollide.size(); ++i) {
      pair<int, int> carind = realCollide[i].getpairind();
      car1 = carind.first;
      car2 = carind.second;
      // cout.precision(8);
      cout << "at " << realCollide[i].gettimeCollide() << ' ';
      cout << cars[car1].getID() << " collided with "
           << cars[car2].getID() << '\n';
    }
  }
  cout << "the remaining vehicles are\n";
  if (survivors.empty()) {
    cout << "none\n";
  } else {
    for (int i{0}; i < survivors.size(); ++i) cout << survivors[i] << '\n';
  }
  return 0;
}
