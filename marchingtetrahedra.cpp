#include <inviwo/tnm067lab3/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/network/networklock.h>
#include <inviwo/tnm067lab1/util/interpolationmethods.h>
#include <iostream>
#include <fstream>

namespace inviwo {

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Stable,                // Code state
    Tags::CPU,                        // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.5f, 0.0f, 1.0f) {

    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    isoValue_.setSerializationMode(PropertySerializationMode::All);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        const auto vr = volume_.getData()->dataMap.valueRange;
        isoValue_.setMinValue(static_cast<float>(vr.x));
        isoValue_.setMaxValue(static_cast<float>(vr.y));
        isoValue_.setIncrement(static_cast<float>(glm::abs(vr.y - vr.x) / 50.0));
        isoValue_.set(static_cast<float>(iso * (vr.y - vr.x) + vr.x));
        isoValue_.setCurrentStateAsDefault();
    });
}

void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto& dims = volume->getDimensions();

    const float iso = isoValue_.get();

    util::IndexMapper3D mapVolPosToIndex(dims);

    static constexpr std::array<std::array<size_t, 4>, 6> tetrahedraIds = {
        {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7}, {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}}};

    size3_t pos{};
    for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) {
        for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
            for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
                // The DataPoint index should be the 1D-index for the DataPoint in the cell
                // Use volume->getAsDouble to query values from the volume
                // Spatial position should be between 0 and 1

                // TODO: TASK 2: create a nested for loop to construct the cell
                Cell c;

                // Nested loop to loop over the 8 vertices (2x2x2) inside the cell
                for (int z = 0; z < 2; ++z) {          // Loop over Z-axis (0 to 1)
                    for (int y = 0; y < 2; ++y) {      // Loop over Y-axis (0 to 1)
                        for (int x = 0; x < 2; ++x) {  // Loop over X-axis (0 to 1)

                            // The position within the current cell
                            vec3 posInCell(x, y, z);
                            // Position of the data point within the entire volume
                            vec3 posInVolume(x + pos.x, y + pos.y, z + pos.z);

                            // Normalized postion within the 3D grid
                            // pos is the 3D position for the current cell in the volume 
                            vec3 scaledPosInVolume =  calculateDataPointPos(pos, posInCell, dims); 

                            // Get 1D Index for Volume
                            size_t indexInVolume = mapVolPosToIndex(posInVolume);

                            // Get the 1D Index of the Data Point in the Cell from 3D
                            size_t vertexIndexInCell = calculateDataPointIndexInCell(posInCell);

                            // Get function value
                            double functionValue = volume->getAsDouble(posInVolume);

                            c.dataPoints[vertexIndexInCell].pos = scaledPosInVolume;
                            c.dataPoints[vertexIndexInCell].value = functionValue;
                            c.dataPoints[vertexIndexInCell].indexInVolume = indexInVolume;
                        }
                    }
                }

                // TODO: TASK 3: Subdivide cell into 6 tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;

                // Loop over the 6 tetrahedra
                for (size_t t = 0; t < tetrahedraIds.size(); ++t) {

                    Tetrahedra tetra;
                    
                     // Loop over the 4 vertices of the current tetrahedron
                    for (auto j = 0; j < 4; ++j) {

                        // get the correct vertex using tetrahedraIds[t][j]
                        // Assign the vertex from the cell's data points
                        // Retrieves the vertex data (position, function value, etc.) for the current vertex in the tetrahedron
                        tetra.dataPoints[j] = c.dataPoints[tetrahedraIds[t][j]];
                    }
                    // Add the constructed tetrahedron to the list of tetrahedra
                    tetrahedras.push_back(tetra);

                }


                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // TODO: TASK 4: Calculate case id for each tetrahedra, and add triangles for
                    // each case (use MeshHelper)

                    // Calculate for tetra case index
                    int caseId = 0;

                    for (auto t = 0; t < 4; ++t) {

                        if (tetrahedra.dataPoints[t].value < iso) {
                            caseId += pow(2, t);
                        }
                    }
                    // Four vertices in tetrahedra
                    DataPoint d0 = tetrahedra.dataPoints[0];
                    DataPoint d1 = tetrahedra.dataPoints[1];
                    DataPoint d2 = tetrahedra.dataPoints[2];
                    DataPoint d3 = tetrahedra.dataPoints[3];

                    // Extract triangles
                    switch (caseId) {
                        case 0:
                        case 15:
                            break;
                        case 1:
                        case 14: {
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d0, d1, iso), d0.indexInVolume, d1.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d0, d3, iso), d0.indexInVolume, d3.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d0, d2, iso), d0.indexInVolume, d2.indexInVolume);

                            if (caseId == 14) {     // normal uppåt
                                mesh.addTriangle(v0, v1, v2);
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                            }
                            break;
                        }
                        case 2:
                        case 13: { 
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d1, d0, iso), d1.indexInVolume, d0.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d1, d2, iso), d1.indexInVolume, d2.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d1, d3, iso), d1.indexInVolume, d3.indexInVolume);

                            if ( caseId==13) {     // normal nedåt
                                mesh.addTriangle(v0, v1, v2);

                            } else {
                                mesh.addTriangle(v0, v2, v1);
                            }
                            break;
                        }
                        case 3:
                        case 12: {
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d1, d2, iso), d1.indexInVolume, d2.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d1, d3, iso), d1.indexInVolume, d3.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d0, d3, iso), d0.indexInVolume, d3.indexInVolume);
                            auto v3 = mesh.addVertex(interpolateVertex(d0, d2, iso), d0.indexInVolume, d2.indexInVolume);

                            if (caseId == 12) {     //uppåt
                                mesh.addTriangle(v0, v1, v2);
                                mesh.addTriangle(v0, v2, v3);
                              
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                                mesh.addTriangle(v0, v3, v2);

                            }
                            break;
                        }
                        case 4:
                        case 11: { 
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d2, d0, iso), d2.indexInVolume, d0.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d2, d3, iso), d2.indexInVolume, d3.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d2, d1, iso), d2.indexInVolume, d1.indexInVolume);

                            if (caseId == 11) {     // nedåt
                                mesh.addTriangle(v0, v1, v2);
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                            }
                            break;
                        }
                        case 5:
                        case 10: {
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d0, d1, iso), d0.indexInVolume, d1.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d0, d3, iso), d0.indexInVolume, d3.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d2, d1, iso), d2.indexInVolume, d1.indexInVolume);
                            auto v3 = mesh.addVertex(interpolateVertex(d2, d3, iso), d2.indexInVolume, d3.indexInVolume);

                            if (caseId == 10) {     // nedåt
                                mesh.addTriangle(v0, v1, v2);
                                mesh.addTriangle(v3, v2, v1);
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                                mesh.addTriangle(v3, v1, v2);
                            }
                            break;
                        }
                        case 6:
                        case 9: { 
                            auto v0 = mesh.addVertex(interpolateVertex(d1, d0, iso), d1.indexInVolume, d0.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d2, d0, iso), d2.indexInVolume, d0.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d1, d3, iso), d1.indexInVolume, d3.indexInVolume);
                            auto v3 = mesh.addVertex(interpolateVertex(d2, d3, iso), d2.indexInVolume, d3.indexInVolume);

                            if (caseId == 9) {      // nedåt 
                                mesh.addTriangle(v0, v1, v2);
                                mesh.addTriangle(v2, v1, v3);
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                                mesh.addTriangle(v2, v3, v1);
                            }
                            break;

                        }
                        case 7:
                        case 8: {
                            
                            auto v0 = mesh.addVertex(interpolateVertex(d3, d1, iso), d3.indexInVolume, d1.indexInVolume);
                            auto v1 = mesh.addVertex(interpolateVertex(d3, d0, iso), d3.indexInVolume, d0.indexInVolume);
                            auto v2 = mesh.addVertex(interpolateVertex(d3, d2, iso), d3.indexInVolume, d2.indexInVolume);

                            if (caseId == 8) {      // uppåt
                                mesh.addTriangle(v0, v1, v2);
                            } else {
                                mesh.addTriangle(v0, v2, v1);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    mesh_.setData(mesh.toBasicMesh());
}

int MarchingTetrahedra::calculateDataPointIndexInCell(ivec3 index3D) {
    // TODO: TASK 1: convert 3D point (i, j, k) to 1D index

    int width = 2;
    int height = 2;
    int i = index3D.x;
    int j = index3D.y;
    int k = index3D.z;

    return i + width * (j + height * k); 
}

vec3 MarchingTetrahedra::calculateDataPointPos(size3_t posVolume, ivec3 posCell, ivec3 dims) {
    // TASK 1
    // Return position of the datapoint within the volume scaled between 0 and 1
    vec3 datapoint;

    datapoint.x = (posVolume.x + posCell.x)/ float(dims.x - 1);
    datapoint.y = (posVolume.y + posCell.y) / float(dims.y - 1);
    datapoint.z = (posVolume.z + posCell.z) / float(dims.z - 1);

    return datapoint;
}

MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    IVW_ASSERT(i0 != i1, "i0 and i1 should not be the same value");
    IVW_ASSERT(i0 != i2, "i0 and i2 should not be the same value");
    IVW_ASSERT(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    const auto a = std::get<0>(vertices_[i0]);
    const auto b = std::get<0>(vertices_[i1]);
    const auto c = std::get<0>(vertices_[i2]);

    const vec3 n = glm::normalize(glm::cross(b - a, c - a));
    std::get<1>(vertices_[i0]) += n;
    std::get<1>(vertices_[i1]) += n;
    std::get<1>(vertices_[i2]) += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        // Normalize the normal of the vertex
        std::get<1>(vertex) = glm::normalize(std::get<1>(vertex));
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    IVW_ASSERT(i != j, "i and j should not be the same value");
    if (j < i) std::swap(i, j);

    auto [edgeIt, inserted] = edgeToVertex_.try_emplace(std::make_pair(i, j), vertices_.size());
    if (inserted) {
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
    }
    return static_cast<std::uint32_t>(edgeIt->second);
}
// Function to calculate the linear interpolation between two points depending on the iso value
vec3 MarchingTetrahedra::interpolateVertex(const DataPoint& p1, const DataPoint& p2, float iso) {

    // Calculate the interpolation factor
    float t = (iso - p1.value) / (p2.value - p1.value);

    // Interpolated vec3
    return p1.pos + t * (p2.pos - p1.pos);
}

}  // namespace inviwo
