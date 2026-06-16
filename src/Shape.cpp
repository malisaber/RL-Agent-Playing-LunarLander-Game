#include "Shape.h"

//Shape::Shape(GLuint shaderProgram, const std::vector<float>& vertices,
//    const std::vector<unsigned int>& indices)
//{
//    this->shaderProgram = shaderProgram;
//    this->col = { 0.0f, 0.0f, 0.0f };
//    model = glm::mat4(1.0f);
//
//
//    glUseProgram(shaderProgram);
//    useEBO = !indices.empty();
//    vertexCount = vertices.size() / 2;  // only positions (x, y)
//    indexCount = indices.size();
//
//    glGenVertexArrays(1, &VAO);
//    glGenBuffers(1, &VBO);
//
//    glBindVertexArray(VAO);
//
//    // Upload vertex data
//    glBindBuffer(GL_ARRAY_BUFFER, VBO);
//    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
//        vertices.data(), GL_STATIC_DRAW);
//
//    // Upload index data if available
//    if (useEBO) {
//        glGenBuffers(1, &EBO);
//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
//            indices.size() * sizeof(unsigned int),
//            indices.data(), GL_STATIC_DRAW);
//    }
//
//    // Vertex attributes (position only)
//    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
//        2 * sizeof(float), (void*)0);
//    glEnableVertexAttribArray(0);
//
//    glBindVertexArray(0);
//}
//
//
//Shape::~Shape() {
//    glDeleteVertexArrays(1, &VAO);
//    glDeleteBuffers(1, &VBO);
//    if (useEBO) {
//        glDeleteBuffers(1, &EBO);
//    }
//}
//
//
//void Shape::color(const glm::vec3& newColor)
//{
//    col = newColor;
//}
//
//
//void Shape::change_shaderProgram(GLuint shaderProgram)
//{
//    this->shaderProgram = shaderProgram;
//}
//
//
//void Shape::move(const glm::vec2& position)
//{
//    // Build model matrix
//    model = glm::translate(model, glm::vec3(position, 0.0f));
//}
//
//
//void Shape::rotate(float rotationDeg)
//{
//    // Build model matrix
//    model = glm::rotate(model, glm::radians(rotationDeg), glm::vec3(0.0f, 0.0f, 1.0f));
//}
//
//
//void Shape::scale(const glm::vec2& scale)
//{
//    // Build model matrix
//    model = glm::scale(model, glm::vec3(scale, 1.0f));
//}
//
//
//void Shape::draw() 
//{   
//    // send color
//    int colorLoc = glGetUniformLocation(shaderProgram, "shapeColor");
//    glUniform3f(colorLoc, col.r, col.g, col.b);
//
//    // sent the moddel to GPU
//    int modelLoc = glGetUniformLocation(shaderProgram, "model");
//    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
//
//    this->model = glm::mat4(1.0f);
//
//
//
//    // draw
//    glBindVertexArray(VAO);
//    if (useEBO)
//    {
//        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
//    }
//    else 
//    {
//        glDrawArrays(GL_TRIANGLES, 0, vertexCount);
//    }
//}
//
