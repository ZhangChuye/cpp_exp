#include "constants.h"

class Cylinder{
    public:
        // Constctors
        Cylinder()=default;
        Cylinder(double rad_param, double height_param);
        // Functions
        double volume();
    
    private : 
        double base_radius{1};
        double height{1};

};