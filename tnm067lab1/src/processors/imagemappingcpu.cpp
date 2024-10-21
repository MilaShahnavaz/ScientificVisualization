#include <inviwo/tnm067lab1/processors/imagemappingcpu.h>
#include <inviwo/tnm067lab1/util/scalartocolormapping.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/imageramutils.h>
#include <inviwo/core/util/stdextensions.h>

#include <fmt/format.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ImageMappingCPU::processorInfo_{
    "org.inviwo.ImageMappingCPU",  // Class identifier
    "Image Mapping CPU",           // Display name
    "TNM067",                      // Category
    CodeState::Stable,             // Code state
    Tags::CPU,                     // Tags
};
const ProcessorInfo ImageMappingCPU::getProcessorInfo() const { return processorInfo_; }

ImageMappingCPU::ImageMappingCPU()
    : Processor()
    , inport_("inport", true)
    , outport_("outport", false)
    , numColors_("numColors", "Number of colors", 2, 1, 10)
    , colors_(util::make_array<10>([](auto n) {
        return FloatVec4Property{fmt::format("color{}", n+1), std::format("Color {}", n+1),
                                 util::ordinalColor(n == 0 ? 1.0f : 0.0f, 0.0f, 0.0f, 1.0f)};
    })) {

    addPort(inport_);
    addPort(outport_);

    addProperty(numColors_);
    for (auto& c : colors_) {
        addProperty(c);
    }

    auto colorVisibility = [&]() {
        for (size_t i = 0; i < 10; i++) {
            colors_[i].setVisible(i < numColors_);
        }
    };
    numColors_.onChange(colorVisibility);
    colorVisibility();
}

void ImageMappingCPU::process() {
    auto inImg = inport_.getData();
    
    auto representation =
        std::make_shared<LayerRAMPrecision<glm::u8vec4>>(inImg->getDimensions(), LayerType::Color);
    glm::u8vec4* outPixels = representation->getDataTyped();
    
    util::IndexMapper2D index(inImg->getDimensions());

    ScalarToColorMapping map;
    for (size_t i = 0; i < numColors_.get(); i++) {
        map.addBaseColors(colors_[i].get());
    }

    inImg->getColorLayer()->getRepresentation<LayerRAM>()->dispatch<void>([&](const auto inRep) {
        auto inPixels = inRep->getDataTyped();
        util::forEachPixelParallel(*inRep, [&](size2_t pos) {
            auto i = index(pos);
            float inPixelVal = util::glm_convert_normalized<float>(inPixels[i]);
            outPixels[i] = map.sample(inPixelVal) * 255.f;
        });
    });

    auto layer = std::make_shared<Layer>(representation);
    auto image = std::make_shared<Image>(layer);
    outport_.setData(image);
}

}  // namespace inviwo
