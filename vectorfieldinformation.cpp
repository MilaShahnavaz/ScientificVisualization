#include <inviwo/tnm067lab4/processors/vectorfieldinformation.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/texture/textureunit.h> 
#include <modules/opengl/shader/shaderutils.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

const ProcessorInfo VectorFieldInformation::processorInfo_{
    "org.inviwo.VectorFieldInformation",  // Class identifier
    "Vector Field Information",           // Display name
    "TNM067",                             // Category
    CodeState::Stable,                    // Code state
    Tags::GL,                             // Tags
};
const ProcessorInfo VectorFieldInformation::getProcessorInfo() const { return processorInfo_; }

VectorFieldInformation::VectorFieldInformation()
    : LayerGLProcessor{utilgl::findShaderResource("vectorfieldinformation.frag")}
    , outputType_{"outputType", "Output",
                  OptionPropertyState<Information>{
                      .options = {{"passThrough", "Pass through", Information::PassThrough},
                                  {"magnitude", "Vector magnitude", Information::Magnitude},
                                  {"divergence", "Divergence", Information::Divergence},
                                  {"rotation", "Rotation", Information::Rotation}},
                      .invalidationLevel = InvalidationLevel::InvalidResources}}
    , outputTypeStr_{"outputTypeStr", "Output", "Pass Through", InvalidationLevel::Valid}
    , minS_{"min", "Input Min Value", "", InvalidationLevel::Valid}
    , maxS_{"max", "Input Max Value", "", InvalidationLevel::Valid} {

    outputTypeStr_.setReadOnly(true);
    outputType_.setCurrentStateAsDefault();
    minS_.setReadOnly(true);
    maxS_.setReadOnly(true);

    addProperties(outputType_, outputTypeStr_, minS_, maxS_);

    outputType_.onChange([this]() { outputTypeStr_.set(outputType_.getSelectedDisplayName()); });
}

void VectorFieldInformation::initializeResources() {
    const static std::string outputKey = "OUTPUT(texCoord_)";
    std::string output = "";

    switch (outputType_.get()) {
        case Information::Rotation:
            output = "rotation(texCoord_.xy);";
            break;
        case Information::Divergence:
            output = "divergence(texCoord_.xy);";
            break;
        case Information::Magnitude:
            output = "magnitude(texCoord_.xy);";
            break;
        case Information::PassThrough:
        default:
            output = "passThrough(texCoord_.xy);";
            break;
    }

    shader_.getFragmentShaderObject()->addShaderDefine(outputKey, output);
    shader_.build();
}

void VectorFieldInformation::preProcess(TextureUnitContainer& cont, const Layer& input,
                                        Layer& output) {
    if (inport_.isChanged()) {
        auto&& [min, max] = util::layerMinMax(&input, IgnoreSpecialValues::Yes);

        minS_.set(toString(min));
        maxS_.set(toString(max));
    }
}

}  // namespace inviwo
