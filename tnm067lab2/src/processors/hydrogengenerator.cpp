#include <inviwo/tnm067lab2/processors/hydrogengenerator.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/util/volumeramutils.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/base/algorithm/dataminmax.h>
#include <cmath>
#include <numbers>

namespace inviwo {

const ProcessorInfo HydrogenGenerator::processorInfo_{
    "org.inviwo.HydrogenGenerator",  // Class identifier
    "Hydrogen Generator",            // Display name
    "TNM067",                        // Category
    CodeState::Stable,               // Code state
    Tags::CPU,                       // Tags
};

const ProcessorInfo HydrogenGenerator::getProcessorInfo() const { return processorInfo_; }

HydrogenGenerator::HydrogenGenerator()
    : Processor(), volume_("volume"), size_("size_", "Volume Size", 16, 4, 256) {
    addPort(volume_);
    addProperty(size_);
}

void HydrogenGenerator::process() {
    auto ram = std::make_shared<VolumeRAMPrecision<float>>(size3_t(size_));
    auto vol = std::make_shared<Volume>(ram);

    auto data = ram->getDataTyped();
    util::IndexMapper3D index(ram->getDimensions());

    util::forEachVoxel(*ram, [&](const size3_t& pos) {
        vec3 cartesian = idTOCartesian(pos);
        data[index(pos)] = static_cast<float>(eval(cartesian));
    });

    auto minMax = util::volumeMinMax(ram.get());
    vol->dataMap.dataRange = vol->dataMap.valueRange = dvec2(minMax.first.x, minMax.second.x);

    volume_.setData(vol);
}

vec3 HydrogenGenerator::cartesianToSpherical(vec3 cartesian) {
    vec3 sph{cartesian};

    // TASK 1: implement conversion using the equations in the lab script
    double r = sqrt(pow(cartesian.x, 2) + pow(cartesian.y, 2) + pow(cartesian.z, 2));
    
    if (r < 0.00001) return vec3{0.0};

    double theta = acos(cartesian.z / r);
    double phi = atan2(cartesian.y, cartesian.x);

    sph.r = r;
    sph.t = theta;
    sph.p = phi;

    return sph;
}

double HydrogenGenerator::eval(vec3 cartesian) {
    // Get spherical coordinates
    vec3 spherical = cartesianToSpherical(cartesian);

    // TASK 2: Evaluate wave function

    double part1 = 1 / (81 * sqrt(6 * M_PI));
    double part2 = 1;
    double part3 = pow(spherical.r, 2);
    double part4 = exp(-spherical.r / 3);
    double part5 = 3 * pow(cos(spherical.t), 2) - 1;

    double density = pow(part1 * part2 * part3 * part4 * part5, 2);

    return density;
}

vec3 HydrogenGenerator::idTOCartesian(size3_t pos) {
    vec3 p(pos);
    p /= size_ - 1;
    return p * (36.0f) - 18.0f;
}

}  // namespace inviwo
