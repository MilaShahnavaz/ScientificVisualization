#include <inviwo/tnm067lab4/processors/lineintegralconvolution.h>

#include <inviwo/core/util/formats.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/texture/textureutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LineIntegralConvolution::processorInfo_{
    "org.inviwo.tnm067.LineIntegralConvolution",  // Class identifier
    "Line Integral Convolution",           // Display name
    "TNM067",                              // Category
    CodeState::Stable,                     // Code state
    Tags::GL,                              // Tags
};
const ProcessorInfo LineIntegralConvolution::getProcessorInfo() const { return processorInfo_; }

LineIntegralConvolution::LineIntegralConvolution()
    : LayerGLProcessor{utilgl::findShaderResource("lineintegralconvolution.frag")}
    , noiseTexture_{"noiseTexture",
                    "2D noise layer which will be convoluted along the vector field"_help}
    , steps_{"nSteps", "Steps", 20, 3, 100}
    , stepSize_{"stepSize", "stepSize", 0.003f, 0.0001f, 0.01f, 0.0001f} {

    outport_.setHelp("Resulting layer with grayscale LIC (single channel)"_help);
    addPort(noiseTexture_);
    addProperties(steps_, stepSize_);

    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidOutput); });
}

void LineIntegralConvolution::preProcess(TextureUnitContainer& cont, const Layer&, Layer&) {
    utilgl::bindAndSetUniforms(shader_, cont, noiseTexture_);
    utilgl::setUniforms(shader_, steps_, stepSize_);
}

LayerConfig LineIntegralConvolution::outputConfig(const Layer& input) const {
    const DataFormatBase* format = input.getDataFormat();

    return input.config().updateFrom({
        .format = DataFormatBase::get(format->getNumericType(), 1, format->getPrecision()),
        .swizzleMask = swizzlemasks::defaultData(1),
        .dataRange = dvec2{0.0, 1.0},
        .valueRange = dvec2{0.0, 1.0},
    });
}


}  // namespace inviwo
