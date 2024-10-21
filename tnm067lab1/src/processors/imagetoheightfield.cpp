#include <inviwo/tnm067lab1/processors/imagetoheightfield.h>
#include <inviwo/tnm067lab1/util/scalartocolormapping.h>
#include <inviwo/core/util/imageramutils.h>
#include <inviwo/core/datastructures/image/layerram.h>

namespace inviwo {

const ProcessorInfo ImageToHeightfield::processorInfo_{
    "org.inviwo.ImageToHeightfield",  // Class identifier
    "ImageToHeightfield",             // Display name
    "TNM067",                         // Category
    CodeState::Stable,                // Code state
    Tags::CPU,                        // Tags
};

const ProcessorInfo ImageToHeightfield::getProcessorInfo() const { return processorInfo_; }

ImageToHeightfield::ImageToHeightfield()
    : Processor()
    , imageInport_("imageInport", true)
    , meshOutport_("meshOutport")
    , heightScaleFactor_("heightScaleFactor", "Height Scale Factor", 1.0f, 0.001f, 2.0f, 0.001f)
    , numColors_("numColors", "Number of colors", 2, 1, 10)
    , colors_(util::make_array<10>([](auto n) {
        return FloatVec4Property{fmt::format("color{}", n+1), std::format("Color {}", n+1),
                                 util::ordinalColor(n == 0 ? 1.0f : 0.0f, 0.0f, 0.0f, 1.0f)};
    })) {

    addPort(imageInport_);
    addPort(meshOutport_);
    addProperty(heightScaleFactor_);

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

namespace {
using HFMesh = TypedMesh<buffertraits::PositionsBuffer, buffertraits::NormalBuffer,
                         buffertraits::ColorsBuffer>;

void addFace(std::vector<HFMesh::Vertex>& vertices, std::vector<unsigned int>& indices,
             const vec3& c1, const vec3& c2, const vec3& c3, const vec3& c4, const vec3& normal,
             const vec4& color) {

    unsigned int startID = static_cast<unsigned int>(vertices.size());
    vertices.emplace_back(c1, normal, color);
    vertices.emplace_back(c2, normal, color);
    vertices.emplace_back(c3, normal, color);
    vertices.emplace_back(c4, normal, color);

    indices.insert(indices.end(),
                   {startID + 0, startID + 1, startID + 2, startID + 0, startID + 2, startID + 3});
}

std::shared_ptr<Mesh> buildMesh(const LayerRAM& image, const ScalarToColorMapping& map,
                                float scaleFactor) {
    const auto dims = image.getDimensions();

    auto mesh = std::make_shared<HFMesh>();
    auto& indices =
        mesh->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)->getDataContainer();

    std::vector<HFMesh::Vertex> vertices;

    const auto bufferSize = 24 * dims.x * dims.y;
    indices.reserve(bufferSize);
    vertices.reserve(bufferSize);

    const vec2 cellSize = 1.0f / vec2(dims);
    util::forEachPixel(image, [&](const size2_t& pos) {
        const vec2 origin2D = vec2(pos) * cellSize;
        const vec3 origin(origin2D.x, 0.0f, origin2D.y);

        // TODO: sample image
        const float imageValue = image.getAsDouble(pos);    //reads the intensity value of a pixel at position pos from the input image
        const float height = imageValue * scaleFactor;  //if the intensity is high, the height will be larger, and if it’s low, the height will be smaller

        const vec4 color =
            vec4(map.sample(imageValue));  // takes the intensity value of a pixel (imageValue) and
                                           // converts it into a color


        // Box Corners
        const auto zero = origin + vec3(0.0f, 0.0f, 0.0f);
        const auto px = origin + vec3(cellSize.x, 0.0f, 0.0f);
        const auto pz = origin + vec3(0.0f, 0.0f, cellSize.y);
        const auto py = origin + vec3(0.0f, height, 0.0f);
        const auto pxpy = origin + vec3(cellSize.x, height, 0.0f);
        const auto pxpz = origin + vec3(cellSize.x, 0.0f, cellSize.y);
        const auto pypz = origin + vec3(0.0f, height, cellSize.y);
        const auto pxpypz = origin + vec3(cellSize.x, height, cellSize.y);

        // Box Normals
        constexpr auto down = vec3(0.0f, -1.0f, 0.0f);
        constexpr auto up = vec3(0.0f, 1.0f, 0.0f);
        constexpr auto left = vec3(-1.0f, 0.0f, 0.0f);
        constexpr auto right = vec3(1.0f, 0.0f, 0.0f);
        constexpr auto front = vec3(0.0f, 0.0f, -1.0f);
        constexpr auto back = vec3(0.0f, 0.0f, 1.0f);

        addFace(vertices, indices, zero, px, pxpz, pz, down, color);       // Bottom face
        addFace(vertices, indices, py, pypz, pxpypz, pxpy, up, color);     // Top face
        addFace(vertices, indices, zero, pz, pypz, py, left, color);       // Left face
        addFace(vertices, indices, px, pxpy, pxpypz, pxpz, right, color);  // Right face
        addFace(vertices, indices, zero, py, pxpy, px, front, color);      // Front face
        addFace(vertices, indices, pz, pxpz, pxpypz, pypz, back, color);   // Back face
    });

    mesh->addVertices(vertices);

    return mesh;
}

}  // namespace

void ImageToHeightfield::process() {
    const auto layer = imageInport_.getData()->getColorLayer()->getRepresentation<LayerRAM>();

    ScalarToColorMapping map;
    for (size_t i = 0; i < numColors_.get(); i++) {
        map.addBaseColors(colors_[i].get());
    }

    const auto mesh = buildMesh(*layer, map, heightScaleFactor_);

    meshOutport_.setData(mesh);
}

}  // namespace inviwo
