#include <inviwo/tnm067lab1/util/scalartocolormapping.h>

namespace inviwo {

void ScalarToColorMapping::clearColors() { baseColors_.clear(); }
void ScalarToColorMapping::addBaseColors(vec4 color) { baseColors_.push_back(color); }

vec4 ScalarToColorMapping::sample(float t) const {
    if (baseColors_.size() == 0) return vec4(t);
    if (baseColors_.size() == 1) return vec4(baseColors_[0]);
    if (t <= 0) return vec4(baseColors_.front());   //return first color
    if (t >= 1) return vec4(baseColors_.back());    //return last color

    // TODO: use t to select which two base colors to interpolate in-between
    // t tells us how far along the range of colors we are
    // since the number of basecolors can differ, we need to scale/stretch t to match the number of intervals between the colors
    
    // TODO: Interpolate colors in baseColors_ and set dummy color to result
    float scaled_t = t * (baseColors_.size() - 1);  // för att beräkna antalet intervall 
    
    float first = floor(scaled_t);
    float second = ceil(scaled_t);

    float factor = scaled_t - first;
    vec4 finalColor = baseColors_[first] * (1.0f - factor) + baseColors_[second] * factor;

    return finalColor;
}

}  // namespace inviwo
