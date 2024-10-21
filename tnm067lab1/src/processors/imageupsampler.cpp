#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <inviwo/tnm067lab1/processors/imageupsampler.h>
#include <inviwo/tnm067lab1/util/interpolationmethods.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/imageramutils.h>

namespace inviwo {

namespace detail {

template <typename T>
void upsample(ImageUpsampler::IntepolationMethod method, const LayerRAMPrecision<T>& inputImage,
              LayerRAMPrecision<T>& outputImage) {
    using F = typename float_type<T>::type;

    const size2_t inputSize = inputImage.getDimensions();
    const size2_t outputSize = outputImage.getDimensions();

    const T* inPixels = inputImage.getDataTyped();
    T* outPixels = outputImage.getDataTyped();

    auto inIndex = [&inputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(inputSize - size2_t(1)));
        return pos.x + pos.y * inputSize.x;
    };
    auto outIndex = [&outputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(outputSize - size2_t(1)));
        return pos.x + pos.y * outputSize.x;
    };

    util::forEachPixel(outputImage, [&](ivec2 outImageCoords) {
        // outImageCoords: Exact pixel coordinates in the output image currently writing to
        // inImageCoords: Relative coordinates of outImageCoords in the input image, might be
        // between pixels
        dvec2 inImageCoords =
            ImageUpsampler::convertCoordinate(outImageCoords, inputSize, outputSize);

        T finalColor(0);

        // DUMMY COLOR, remove or overwrite this bellow
        finalColor = inPixels[inIndex(
            glm::clamp(size2_t(outImageCoords), size2_t(0), size2_t(inputSize - size2_t(1))))];

        switch (method) {
            case ImageUpsampler::IntepolationMethod::PiecewiseConstant: {   //nearest neighbour
                // Task 6
                finalColor = inPixels[inIndex(round(inImageCoords-dvec2(0.5)))];
                break;
            }
            case ImageUpsampler::IntepolationMethod::Bilinear: {

                //  base pixel in the bottom left corner (0,0)
                dvec2 pixel = inImageCoords - dvec2(0.5);
                ivec2 basePixel = floor(pixel);

                // define surrounding pixels
                std::array<T, 4> neighbouringPixels = {
                    inPixels[inIndex(basePixel)],                // pixel (x,y)
                    inPixels[inIndex(basePixel + ivec2(1, 0))],  // pixel (x+1,y)
                    inPixels[inIndex(basePixel + ivec2(0, 1))],  // pixel (x,y+1)
                    inPixels[inIndex(basePixel + ivec2(1, 1))],  // pixel (x+1,y+1)
                };

                // Calculate the distance of the current point from the base pixel
                double x_dist = pixel.x - basePixel.x;
                double y_dist = pixel.y - basePixel.y;

                // Perform bilinear interpolation using the 4 neighboring pixels
                finalColor = TNM067::Interpolation::bilinear(neighbouringPixels, x_dist, y_dist);

                break;
            }
            case ImageUpsampler::IntepolationMethod::Biquadratic: {
                //  base pixel in the bottom left corner (0,0)
                 dvec2 pixel = inImageCoords - dvec2(0.5);
                 ivec2 basePixel = floor(pixel);

                // define surrounding pixels
                std::array<T, 9> neighbouringPixels = {
                    // bottom
                    inPixels[inIndex(basePixel)],                // pixel (x,y)
                    inPixels[inIndex(basePixel + ivec2(1, 0))],  // pixel (x+1,y)
                    inPixels[inIndex(basePixel + ivec2(2, 0))],  // pixel (x+2,y)

                    // middle
                    inPixels[inIndex(basePixel + ivec2(0, 1))],  // pixel (x,y+1)
                    inPixels[inIndex(basePixel + ivec2(1, 1))],  // pixel (x+1,y+1)
                    inPixels[inIndex(basePixel + ivec2(2, 1))],  // pixel (x+2,y+1)

                    // top 
                    inPixels[inIndex(basePixel + ivec2(0, 2))],  // pixel (x,y+2)
                    inPixels[inIndex(basePixel + ivec2(1, 2))],  // pixel (x+1,y+2)
                    inPixels[inIndex(basePixel + ivec2(2, 2))],  // pixel (x+2,y+2)

                };

                 // Calculate the distance of the current point from the base pixel
                double x_dist = (pixel.x - basePixel.x) / 2;    // Distance between grid is now 0-2 so we normalize
                double y_dist = (pixel.y - basePixel.y) / 2;

                finalColor = TNM067::Interpolation::biQuadratic(neighbouringPixels, x_dist, y_dist);

                break;
            }
            case ImageUpsampler::IntepolationMethod::Barycentric: {

                 dvec2 pixel = inImageCoords - dvec2(0.5);
                 ivec2 basePixel = floor(pixel);
                // define surrounding pixels
                std::array<T, 4> neighbouringPixels = {
                    inPixels[inIndex(basePixel)],                // pixel (x,y)
                    inPixels[inIndex(basePixel + ivec2(1, 0))],  // pixel (x+1,y)
                    inPixels[inIndex(basePixel + ivec2(0, 1))],  // pixel (x,y+1)
                    inPixels[inIndex(basePixel + ivec2(1, 1))],  // pixel (x+1,y+1)
                };
                double x_dist = pixel.x - basePixel.x;
                double y_dist = pixel.y - basePixel.y;

                finalColor = TNM067::Interpolation::barycentric(neighbouringPixels, x_dist, y_dist);
                break;
            }
            default:
                break;
        }

        outPixels[outIndex(outImageCoords)] = finalColor;
    });
}

}  // namespace detail

const ProcessorInfo ImageUpsampler::processorInfo_{
    "org.inviwo.imageupsampler",  // Class identifier
    "Image Upsampler",            // Display name
    "TNM067",                     // Category
    CodeState::Experimental,      // Code state
    Tags::CPU,                    // Tags
};
const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

ImageUpsampler::ImageUpsampler()
    : Processor()
    , inport_("inport", true)
    , outport_("outport", true)
    , interpolationMethod_("interpolationMethod", "Interpolation Method",
                           {
                               {"piecewiseconstant", "Piecewise Constant (Nearest Neighbor)",
                                IntepolationMethod::PiecewiseConstant},
                               {"bilinear", "Bilinear", IntepolationMethod::Bilinear},
                               {"quadratic", "Quadratic", IntepolationMethod::Biquadratic},
                               {"barycentric", "Barycentric", IntepolationMethod::Barycentric},
                           }) {
    addPort(inport_);
    addPort(outport_);
    addProperty(interpolationMethod_);
}

void ImageUpsampler::process() {
    auto inputImage = inport_.getData();
    if (inputImage->getDataFormat()->getComponents() != 1) {
        LogError("The ImageUpsampler processor does only support single channel images");
    }

    auto inSize = inport_.getData()->getDimensions();
    auto outDim = outport_.getDimensions();

    auto outputImage = std::make_shared<Image>(outDim, inputImage->getDataFormat());
    outputImage->getColorLayer()->setSwizzleMask(inputImage->getColorLayer()->getSwizzleMask());
    outputImage->getColorLayer()
        ->getEditableRepresentation<LayerRAM>()
        ->dispatch<void, dispatching::filter::Scalars>([&](auto outRep) {
            auto inRep = inputImage->getColorLayer()->getRepresentation<LayerRAM>();
            detail::upsample(interpolationMethod_.get(), *(const decltype(outRep))(inRep), *outRep);
        });

    outport_.setData(outputImage);
}

dvec2 ImageUpsampler::convertCoordinate(ivec2 outImageCoords, size2_t inputSize,
                                        size2_t outputSize) {
    // TODO implement
    dvec2 c(outImageCoords);    // integer output pixel coordinates into a dvec2

    // TASK 5: Convert the outImageCoords to its coordinates in the input image

    dvec2 scaleFactor = dvec2(inputSize) / dvec2(outputSize);   // scaling factor between the input and output image 
    return (c * scaleFactor);   // scaling factor to the pixel coordinates c from the output image to convert them to the corresponding coordinates in the input image
}

}  // namespace inviwo
