#include "utils/structs.glsl"

uniform sampler2D inport;
uniform ImageParameters inportParameters;
uniform ImageParameters outportParameters;

float passThrough(vec2 coord){
    return texture(inport, coord).x;
}

float magnitude( vec2 coord ){
    //TASK 1: find the magnitude of the vectorfield at the position coords
    // velo is the position x,y of the vector field  
    vec2 velo = texture(inport, coord.xy).xy;
    float res = sqrt(pow(velo.x,2) + pow(velo.y,2));    // eq2
    return res;
} // it can be extended using vec3 with coord.z

float divergence(vec2 coord){
    //TASK 2: find the divergence of the vectorfield at the position coords
    vec2 pixelSize = inportParameters.reciprocalDimensions;

    vec2 right = texture(inport, vec2(coord.x + pixelSize.x, coord.y)).xy;  // samples the vector field at a position one pixel to the right of the current coordinate
    vec2 left = texture(inport, vec2(coord.x - pixelSize.x, coord.y)).xy;

    vec2 above = texture(inport, vec2(coord.x, coord.y + pixelSize.y)).xy;
    vec2 below = texture(inport, vec2(coord.x, coord.y - pixelSize.y)).xy;

    vec2 dvdx = (right - left) / (2 * pixelSize.x);      // eq5
    vec2 dvdy = (above - below) / (2 * pixelSize.y);      // eq6

    return dvdx.x + dvdy.y;
    
}

float rotation(vec2 coord){
    //TASK 3: find the curl of the vectorfield at the position coords
    vec2 pixelSize = inportParameters.reciprocalDimensions;

    vec2 right = texture(inport, vec2(coord.x + pixelSize.x, coord.y)).xy;
    vec2 left = texture(inport, vec2(coord.x - pixelSize.x, coord.y)).xy;

    vec2 above = texture(inport, vec2(coord.x, coord.y + pixelSize.y)).xy;
    vec2 below = texture(inport, vec2(coord.x, coord.y - pixelSize.y)).xy;

    vec2 dvdx = (right - left) / (2 * pixelSize.x);
    vec2 dvdy = (above - below) / (2 * pixelSize.y);

    return dvdx.y - dvdy.x;
}

void main(void) {
    vec2 texCoords = gl_FragCoord.xy * outportParameters.reciprocalDimensions;

    float v = OUTPUT(texCoords);

    FragData0 = vec4(v);
}
