#pragma once

const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
uniform mat4 model;  // shape transformation
void main()
{
    gl_Position = model * vec4(aPos, 0.0, 1.0);
}
)";

// Fragment shader source
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 shapeColor;
void main()
{
    FragColor = vec4(shapeColor, 1.0);
}
)";