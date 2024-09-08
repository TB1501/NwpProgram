#include "statika.h"
#include "iznimke.h"
#include <iostream>
#include <vector>

int main() {
    try {

        // Creating objects for forces, moments, and uniform loads

        statika::Moment moment;
        moment.M = 0.0; // Example moment value
        moment.x = 0.0;     // Example moment position



        statika::Force force;
        force.Fx = 0;    // Example force in x direction
        force.Fy = 5;   // Example force in y direction
        force.Fx_x = 0;    // Example position of force in x direction
        force.Fy_x = 7;    // Example position of force in y direction

        statika::UniformLoad uniformLoad;
        uniformLoad.q = 2.0;    // Example uniformly distributed load
        uniformLoad.x1 = 2.0;     // Start position of the uniform load
        uniformLoad.x2 = 8.0;    // End position of the uniform load




        // Creating a Beam object with a length of 10 units
        statika::Beam beam;
        beam.L = 10;

        statika::StaticEquilibrium proracun;

        if (beam.L <= 0) throw InvalidBeamLength();
        if (moment.x < 0 || force.Fx_x < 0 || force.Fy_x < 0 || uniformLoad.x1 < 0 || uniformLoad.x2 < 0)
            throw NegativeValueException();
        if (moment.x > beam.L || force.Fx_x > beam.L || force.Fy_x > beam.L || uniformLoad.x1 > beam.L || uniformLoad.x2 > beam.L)
            throw InvalidForcePosition();

        // Calculate reactions at supports A and B
        double FAx = proracun.reactionOnSupportAx(force);
        double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
        double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);



        // Display reactions
        std::cout << "Reaction at support A in x direction (FAx): " << FAx << std::endl;
        std::cout << "Reaction at support A in y direction (FAy): " << FAy << std::endl;
        std::cout << "Reaction at support B in y direction (FBy): " << FBy << std::endl;

        // Calculate internal moments
        proracun.calcInternalMoment(beam, moment, force, uniformLoad);

        // Get internal moments
        const auto& internalMoments = proracun.getInternalMoments();

        // Display internal moments
        std::cout << "Internal moments on the beam:" << std::endl;
        for (const auto& internalMoment : internalMoments) {
            std::cout << "Moment: " << internalMoment.first << " at position: " << internalMoment.second << std::endl;
        }


        // Display the maximum moment on the beam
        std::cout << "Max moment on the beam:" << proracun.getMaxMomentValue(internalMoments) << std::endl;
    }
    catch (const InvalidBeamLength& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    catch (const InvalidForcePosition& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    catch (const NegativeValueException& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }


    return 0;
}
