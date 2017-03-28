#include "wave/wave_test.hpp"
#include "wave/utils/data.hpp"
#include "wave/utils/config.hpp"

#define TEST_CONFIG "tests/data/config.yaml"


TEST(Utils_config_ConfigParam, constructor) {
    wave::ConfigParam param;

    ASSERT_EQ(wave::TYPE_NOT_SET, param.type);
    ASSERT_EQ("", param.key);
    ASSERT_FALSE(param.optional);
    ASSERT_EQ(NULL, param.data);
}

TEST(Utils_config_ConfigParser, constructor) {
    wave::ConfigParser parser;

    ASSERT_FALSE(parser.configured);
    ASSERT_FALSE(parser.loaded);
}

TEST(Utils_config_ConfigParser, addParam) {
    bool b;
    int i;
    float f;
    double d;
    std::string s;

    std::vector<bool> b_array;
    std::vector<int> i_array;
    std::vector<float> f_array;
    std::vector<double> d_array;
    std::vector<std::string> s_array;

    wave::Vec2 vec2;
    wave::Vec3 vec3;
    wave::Vec4 vec4;
    wave::VecX vecx;

    wave::Mat2 mat2;
    wave::Mat3 mat3;
    wave::Mat4 mat4;
    wave::MatX matx;
    cv::Mat cvmat;

    wave::ConfigParser parser;

    parser.addParam("bool", &b);
    parser.addParam("int", &i);
    parser.addParam("float", &f);
    parser.addParam("double", &d);
    parser.addParam("string", &s);

    parser.addParam("bool_array", &b_array);
    parser.addParam("int_array", &i_array);
    parser.addParam("float_array", &f_array);
    parser.addParam("double_array", &d_array);
    parser.addParam("string_array", &s_array);

    parser.addParam("vector2", &vec2);
    parser.addParam("vector3", &vec3);
    parser.addParam("vector4", &vec4);
    parser.addParam("vector", &vecx);

    parser.addParam("matrix2", &mat2);
    parser.addParam("matrix3", &mat3);
    parser.addParam("matrix4", &mat4);
    parser.addParam("matrix", &matx);
    parser.addParam("matrix", &cvmat);

    ASSERT_EQ(19, (int) parser.params.size());
    ASSERT_EQ(wave::BOOL, parser.params[0].type);
    ASSERT_EQ("bool", parser.params[0].key);
    ASSERT_TRUE(parser.params[0].data != NULL);
}

TEST(Utils_config_ConfigParser, getYamlNode) {
    YAML::Node node1, node2;
    wave::ConfigParser parser;

    parser.load(TEST_CONFIG);

    parser.getYamlNode("level3.a.b.c", node1);
    ASSERT_EQ(3, node1.as<int>());

    parser.getYamlNode("float", node2);
    ASSERT_FLOAT_EQ(2.2, node2.as<float>());
}

TEST(Utils_config_ConfigParser, loadPrimitive) {
    int i;
    float f;
    double d;
    std::string s;
    wave::ConfigParser parser;
    wave::ConfigParam param;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);

    // INTEGER
    param.optional = false;
    param.type = wave::INT;
    param.key = "int";
    param.data = &i;
    parser.loadPrimitive(param);
    ASSERT_EQ(1, i);

    // FLOAT
    param.optional = false;
    param.type = wave::FLOAT;
    param.key = "float";
    param.data = &f;
    parser.loadPrimitive(param);
    ASSERT_FLOAT_EQ(2.2, f);

    // DOUBLE
    param.optional = false;
    param.type = wave::DOUBLE;
    param.key = "double";
    param.data = &d;
    parser.loadPrimitive(param);
    ASSERT_FLOAT_EQ(3.3, d);

    // STRING
    param.optional = false;
    param.type = wave::STRING;
    param.key = "string";
    param.data = &s;
    parser.loadPrimitive(param);
    ASSERT_EQ("hello world!", s);
}

TEST(Utils_config_ConfigParser, loadArray) {
    std::vector<bool> b_array;
    std::vector<int> i_array;
    std::vector<float> f_array;
    std::vector<double> d_array;
    std::vector<std::string> s_array;
    wave::ConfigParser parser;
    wave::ConfigParam param;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);

    // BOOL ARRAY
    param.optional = false;
    param.type = wave::BOOL_ARRAY;
    param.key = "bool_array";
    param.data = &b_array;
    parser.loadArray(param);

    ASSERT_TRUE(b_array[0]);
    ASSERT_FALSE(b_array[1]);
    ASSERT_TRUE(b_array[2]);
    ASSERT_FALSE(b_array[3]);

    // INTEGER
    param.optional = false;
    param.type = wave::INT_ARRAY;
    param.key = "int_array";
    param.data = &i_array;
    parser.loadArray(param);

    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(i + 1, i_array[i]);
    }

    // FLOAT
    param.optional = false;
    param.type = wave::FLOAT_ARRAY;
    param.key = "float_array";
    param.data = &f_array;
    parser.loadArray(param);

    for (int i = 0; i < 4; i++) {
        ASSERT_FLOAT_EQ((i + 1) * 1.1, f_array[i]);
    }

    // DOUBLE
    param.optional = false;
    param.type = wave::DOUBLE_ARRAY;
    param.key = "double_array";
    param.data = &d_array;
    parser.loadArray(param);

    for (int i = 0; i < 4; i++) {
        ASSERT_FLOAT_EQ((i + 1) * 1.1, d_array[i]);
    }

    // STRING
    param.optional = false;
    param.type = wave::STRING_ARRAY;
    param.key = "string_array";
    param.data = &s_array;
    parser.loadArray(param);

    ASSERT_EQ("1.1", s_array[0]);
    ASSERT_EQ("2.2", s_array[1]);
    ASSERT_EQ("3.3", s_array[2]);
    ASSERT_EQ("4.4", s_array[3]);
}

TEST(Utils_config_ConfigParser, loadVector) {
    wave::Vec2 vec2;
    wave::Vec3 vec3;
    wave::Vec4 vec4;
    wave::VecX vecx;
    wave::ConfigParser parser;
    wave::ConfigParam param;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);

    // VECTOR 2
    param.optional = false;
    param.type = wave::VEC2;
    param.key = "vector2";
    param.data = &vec2;
    parser.loadVector(param);

    ASSERT_FLOAT_EQ(1.1, vec2(0));
    ASSERT_FLOAT_EQ(2.2, vec2(1));

    // VECTOR 3
    param.optional = false;
    param.type = wave::VEC3;
    param.key = "vector3";
    param.data = &vec3;
    parser.loadVector(param);

    ASSERT_FLOAT_EQ(1.1, vec3(0));
    ASSERT_FLOAT_EQ(2.2, vec3(1));
    ASSERT_FLOAT_EQ(3.3, vec3(2));

    // VECTOR 4
    param.optional = false;
    param.type = wave::VEC4;
    param.key = "vector4";
    param.data = &vec4;
    parser.loadVector(param);

    ASSERT_FLOAT_EQ(1.1, vec4(0));
    ASSERT_FLOAT_EQ(2.2, vec4(1));
    ASSERT_FLOAT_EQ(3.3, vec4(2));
    ASSERT_FLOAT_EQ(4.4, vec4(3));

    // VECTOR X
    param.optional = false;
    param.type = wave::VECX;
    param.key = "vector";
    param.data = &vecx;
    parser.loadVector(param);

    for (int i = 0; i < 9; i++) {
        ASSERT_FLOAT_EQ((i + 1) * 1.1, vecx(i));
    }
}

TEST(Utils_config_ConfigParser, loadMatrix) {
    int index;
    wave::Mat2 mat2;
    wave::Mat3 mat3;
    wave::Mat4 mat4;
    wave::MatX matx;
    cv::Mat cvmat;
    wave::ConfigParser parser;
    wave::ConfigParam param;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);

    // MATRIX 2
    param.optional = false;
    param.type = wave::MAT2;
    param.key = "matrix2";
    param.data = &mat2;
    parser.loadMatrix(param);

    ASSERT_FLOAT_EQ(1.1, mat2(0, 0));
    ASSERT_FLOAT_EQ(2.2, mat2(0, 1));
    ASSERT_FLOAT_EQ(3.3, mat2(1, 0));
    ASSERT_FLOAT_EQ(4.4, mat2(1, 1));

    // MATRIX 3
    param.optional = false;
    param.type = wave::MAT3;
    param.key = "matrix3";
    param.data = &mat3;
    parser.loadMatrix(param);

    index = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, mat3(i, j));
            index++;
        }
    }

    // MATRIX 4
    param.optional = false;
    param.type = wave::MAT4;
    param.key = "matrix4";
    param.data = &mat4;
    parser.loadMatrix(param);

    index = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, mat4(i, j));
            index++;
        }
    }

    // MATRIX X
    param.optional = false;
    param.type = wave::MATX;
    param.key = "matrix";
    param.data = &matx;
    parser.loadMatrix(param);

    index = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, matx(i, j));
            index++;
        }
    }

    // CV MATRIX
    param.optional = false;
    param.type = wave::CVMAT;
    param.key = "matrix";
    param.data = &cvmat;
    parser.loadMatrix(param);

    index = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, cvmat.at<double>(i, j));
            index++;
        }
    }
}

TEST(Utils_config_ConfigParser, load) {
    int retval;
    bool b;
    int i;
    float f;
    double d;
    std::string s;

    std::vector<bool> b_array;
    std::vector<int> i_array;
    std::vector<float> f_array;
    std::vector<double> d_array;
    std::vector<std::string> s_array;

    wave::Vec2 vec2;
    wave::Vec3 vec3;
    wave::Vec4 vec4;
    wave::VecX vecx;

    wave::Mat2 mat2;
    wave::Mat3 mat3;
    wave::Mat4 mat4;
    wave::MatX matx;
    cv::Mat cvmat;

    wave::ConfigParser parser;

    parser.addParam("bool", &b);
    parser.addParam("int", &i);
    parser.addParam("float", &f);
    parser.addParam("double", &d);
    parser.addParam("string", &s);

    parser.addParam("bool_array", &b_array);
    parser.addParam("int_array", &i_array);
    parser.addParam("float_array", &f_array);
    parser.addParam("double_array", &d_array);
    parser.addParam("string_array", &s_array);

    parser.addParam("vector2", &vec2);
    parser.addParam("vector3", &vec3);
    parser.addParam("vector4", &vec4);
    parser.addParam("vector", &vecx);

    parser.addParam("matrix2", &mat2);
    parser.addParam("matrix3", &mat3);
    parser.addParam("matrix4", &mat4);
    parser.addParam("matrix", &matx);
    parser.addParam("matrix", &cvmat);

    retval = parser.load(TEST_CONFIG);
    if (retval != 0) {
        FAIL();
    }

    std::cout << "bool: " << b << std::endl;
    std::cout << "int: " << i << std::endl;
    std::cout << "float: " << f << std::endl;
    std::cout << "double: " << d << std::endl;
    std::cout << "string: " << s << std::endl;
    std::cout << std::endl;

    std::cout << "vector2: " << vec2.transpose() << std::endl;
    std::cout << "vector3: " << vec3.transpose() << std::endl;
    std::cout << "vector4: " << vec4.transpose() << std::endl;
    std::cout << "vector: " << vecx.transpose() << std::endl;
    std::cout << std::endl;

    std::cout << "matrix2: \n" << mat2 << std::endl;
    std::cout << "matrix3: \n" << mat3 << std::endl;
    std::cout << "matrix4: \n" << mat4 << std::endl;
    std::cout << "matrix: \n" << matx << std::endl;
    std::cout << "cvmatrix: \n" << cvmat << std::endl;
    std::cout << std::endl;
}
