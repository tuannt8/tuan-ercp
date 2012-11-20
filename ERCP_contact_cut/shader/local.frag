//
// Fragment shader for procedural bricks
//
// Authors: Dave Baldwin, Steve Koren, Randi Rost
//          based on a shader by Darwyn Peachey
//
// Copyright (c) 2002-2004 3Dlabs Inc. Ltd.
//
// See 3Dlabs-License.txt for license information
//  

varying float LightIntensity;
varying vec3 color;

void main(void)
{
    color *= LightIntensity;
    gl_FragColor = vec4 (color, 1.0);
}