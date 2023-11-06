// AERSP_424_HW2.cpp : Defines the entry point for the application.
//

#include <iostream>
#include <map>
#include <string>
#include <random>
#include <vector>
#include "HW2_Visualizer.h"

struct Airport {
    std::string code;
    std::map<std::string, int> destinations;
};


class Plane {
protected:
    double wait_time;

private:
    double pos;
    double vel;
    double distance;
    double loiter_time;
    bool at_SCE;
    std::string origin;
    std::string destination;

    // Container of airport distances
    std::map<std::string, std::map<std::string, int>> airportDistances = {
        { "SCE", { {"PHL", 160}, {"ORD", 640}, {"EWR", 220} } },
        { "PHL", { {"SCE", 160} } },
        { "ORD", { {"SCE", 640} } },
        { "EWR", { {"SCE", 220} } }
    };

public:
    Plane(const std::string& from, const std::string& to)
        : origin(from), destination(to), at_SCE(false), pos(0), vel(0), distance(0), loiter_time(0), wait_time(0)
    {
        if (airportDistances.find(from) != airportDistances.end()) {
            if (airportDistances[from].find(to) != airportDistances[from].end()) {
                distance = airportDistances[from][to];
            }
        }
    }

    virtual ~Plane() {}

    // Operate function based on flowchart
    void operate(double dt) {
        // std::cout << "Operate Start: Pos: " << pos << ", Vel: " << vel << ", Dist: " << distance
           // << ", Loiter: " << loiter_time << ", Wait: " << wait_time
           // << ", Origin: " << origin << ", Destination: " << destination << std::endl;

        // Loitering time
        if (loiter_time != 0) {
            loiter_time -= dt;
            if (loiter_time < 0) loiter_time = 0;
            return;
        }
        //Waiting time
        if (wait_time != 0) {
            wait_time -= dt;
            if (wait_time < 0) wait_time = 0;
            return;
        }
        // Swapping destination with origin
        if (pos >= distance) {
            at_SCE = (destination == "SCE");
            std::swap(origin, destination);
            if (airportDistances.find(origin) != airportDistances.end()) {
                if (airportDistances[origin].find(destination) != airportDistances[origin].end()) {
                    distance = airportDistances[origin][destination];
                }
                else {
                    std::cerr << "Error: Distance not found for " << origin << " to " << destination << std::endl;
                }
            }
            else {
                std::cerr << "Error: Origin not found in distances map: " << origin << std::endl;
            }
            pos = 0;
            //std::cout << "New Distance After Swapping: " << distance << std::endl;
        }
        pos += vel * (dt / 3600.0);

       // std::cout << "Operate End: Pos: " << pos << ", Vel: " << vel << ", Dist: " << distance
          //  << ", Loiter: " << loiter_time << ", Wait: " << wait_time
          //  << ", Origin: " << origin << ", Destination: " << destination << std::endl;
    }

    // Implementing get and set functions
    double get_pos() const { return pos; }
    double get_vel() const { return vel; }
    double get_loiter_time() const { return loiter_time; }
    std::string get_origin() const { return origin; }
    std::string get_destination() const { return destination; }
    bool get_at_SCE() const { return at_SCE; }

    void set_vel(double velocity) { vel = velocity; }
    void set_loiter_time(double l_time) {
        loiter_time = l_time;
        if (loiter_time < 0) loiter_time = 0;
    }
    void set_wait_time(double w_time) {
        wait_time = w_time;
        if (wait_time < 0) wait_time = 0;
    }

    double distance_to_SCE() const {
        if (destination == "SCE") {
            return distance - pos;
        }
        return -1.0; // Indicates that the destination is not SCE
    }

    virtual double time_on_ground() = 0; // pure virtual function to be implemented by derived classes

    virtual std::string plane_type() {
        return "GA";
    }

    static double draw_from_normal_dist(double m, double sd) {
        std::random_device rd{};
        std::mt19937 gen{ rd() };
        std::normal_distribution<> d{ m, sd };
        return d(gen);
    }
};

class Airliner : public Plane {
private:
    std::string Airline;

public:
    // Constructor
    Airliner(const std::string& airline, const std::string& from, const std::string& to)
        : Plane(from, to), Airline(airline) {}

    // Destructor
    ~Airliner() {}

    // Overridden plane_type function
    std::string plane_type() override {
        return Airline;
    }

    // Overridden time_on_ground function
    double time_on_ground() override {
        double wait_time = draw_from_normal_dist(1800, 600);
        set_wait_time(wait_time); // Set the wait_time of the Plane class
        return wait_time;
    }
};

class GeneralAviation : public Plane {
public:
    // Constructor
    GeneralAviation(const std::string& from, const std::string& to)
        : Plane(from, to) {}

    // Destructor
    ~GeneralAviation() {}

    // Overridden time_on_ground function
    double time_on_ground() override {
        double wait_time = draw_from_normal_dist(600, 60);
        set_wait_time(wait_time); // Set the wait_time of the Plane class
        return wait_time;
    }
};

class ATC {
private:
    std::vector<Plane*> registered_planes; // Container to store pointers to registered planes
    const int MAX_LANDED_PLANE_NUM = 2;
    const int AIRSPACE_DISTANCE = 50;

public:
    // Empty Constructor
    ATC() {}

    // Empty Destructor
    ~ATC() {}

    // Function to register a plane
    void register_plane(Plane* plane) {
        registered_planes.push_back(plane);
    }

    // Function to control traffic based on flowchart
    void control_traffic(double dt) {
        int landed_planes = 0;
        int current_index = 0;

        while (current_index < registered_planes.size()) {
            Plane* plane = registered_planes[current_index];

            landed_planes += static_cast<int>(plane->get_at_SCE());

            if (landed_planes >= MAX_LANDED_PLANE_NUM) {
                current_index = 0;
            }
            else {
                if (plane->get_at_SCE() == 0 &&
                    plane->distance_to_SCE() <= AIRSPACE_DISTANCE &&
                    plane->get_loiter_time() == 0) {

                    plane->set_loiter_time(100);
                }
                current_index++;
            }
        }
        
        for (Plane* plane : registered_planes) {
            plane->operate(dt);
        }
        
    }
};

int main(int argc, char** argv) {
    std::map<std::string, Airport> airports;

    // Initialize the airports with their  codes
    airports["SCE"] = { "SCE", {} };
    airports["PHL"] = { "PHL", {} };
    airports["ORD"] = { "ORD", {} };
    airports["EWR"] = { "EWR", {} };

    // Set the distances for SCE
    airports["SCE"].destinations["PHL"] = 160;
    airports["SCE"].destinations["ORD"] = 640;
    airports["SCE"].destinations["EWR"] = 220;

    // Instantiate plane objects
    Airliner aa1("AA", "SCE", "PHL"), aa2("AA", "SCE", "ORD");
    Airliner ua1("UA", "SCE", "ORD"), ua2("UA", "SCE", "EWR");
    GeneralAviation ga1("SCE", "PHL"), ga2("SCE", "EWR"), ga3("SCE", "ORD");

    // Set the speed for each plane object
    aa1.set_vel(470); aa2.set_vel(500);
    ua1.set_vel(515); ua2.set_vel(480);
    ga1.set_vel(140); ga2.set_vel(160);
    ga3.set_vel(180);

    // Instantiate the ATC object
    ATC atc;

    // Register all the plane objects to the ATC
    atc.register_plane(&aa1); atc.register_plane(&aa2);
    atc.register_plane(&ua1); atc.register_plane(&ua2);
    atc.register_plane(&ga1); atc.register_plane(&ga2);
    atc.register_plane(&ga3);

    // Instantiate the HW2_VIZ object 
    HW2_VIZ viz;

    //timestep between [10, 100]
    int dt = 23;

    while (true) {

        //std::cout << "Before operate: AA1 Pos: " << aa1.get_pos() << ", Vel: " << aa1.get_vel() << ", Dist: " << airports[aa1.get_origin()].destinations[aa1.get_destination()] << "\n";
        aa1.operate(dt);
        //std::cout << "After operate: AA1 Pos: " << aa1.get_pos() << ", Vel: " << aa1.get_vel() << ", Dist: " << airports[aa1.get_origin()].destinations[aa1.get_destination()] << "\n";

        ua1.operate(dt);
        aa2.operate(dt); ua2.operate(dt);
        ga1.operate(dt); ga2.operate(dt);
        ga3.operate(dt);
        
        atc.control_traffic(dt);
        
        //std::cout << "AA1 Position: " << aa1.get_pos() << " miles\n";
        //std::cout << "AA2 Position: " << aa2.get_pos() << " miles\n";
        //std::cout << "UA1 Position: " << ua1.get_pos() << " miles\n";
        //std::cout << "UA2 Position: " << ua2.get_pos() << " miles\n";
        //std::cout << "GA1 Position: " << ga1.get_pos() << " miles\n";
        //std::cout << "GA2 Position: " << ga2.get_pos() << " miles\n";

        // Visualize each plane
        viz.visualize_plane(aa1.plane_type(), aa1.get_origin(), aa1.get_destination(), aa1.get_pos());
        viz.visualize_plane(aa2.plane_type(), aa2.get_origin(), aa2.get_destination(), aa2.get_pos());
        viz.visualize_plane(ua1.plane_type(), ua1.get_origin(), ua1.get_destination(), ua1.get_pos());
        viz.visualize_plane(ua2.plane_type(), ua2.get_origin(), ua2.get_destination(), ua2.get_pos());
        viz.visualize_plane("GA", ga1.get_origin(), ga1.get_destination(), ga1.get_pos());
        viz.visualize_plane("GA", ga2.get_origin(), ga2.get_destination(), ga2.get_pos());
        viz.visualize_plane("GA", ga3.get_origin(), ga3.get_destination(), ga3.get_pos());

        // Call the update function of viz object
        viz.update(dt);
    }

    return 0;
}