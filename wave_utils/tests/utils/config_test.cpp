#include "wave/wave_test.hpp"
#include "wave/utils/data.hpp"
#include "wave/utils/config.hpp"

const auto TEST_CONFIG = "tests/data/config.yaml";


TEST(Utils_config_ConfigParser, constructor) {
    wave::ConfigParser parser;

    ASSERT_FALSE(parser.config_loaded);
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
    ASSERT_EQ("bool", parser.params[0]->key);
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

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);
    parser.config_loaded = true;

    // INTEGER
    wave::ConfigParam<int> int_param{"int", &i, false};
    parser.loadParam(int_param);
    ASSERT_EQ(1, i);

    // FLOAT
    wave::ConfigParam<float> float_param{"float", &f, false};
    parser.loadParam(float_param);
    ASSERT_FLOAT_EQ(2.2, f);

    // DOUBLE
    wave::ConfigParam<double> double_param{"double", &d, false};
    parser.loadParam(double_param);
    ASSERT_FLOAT_EQ(3.3, d);

    // STRING
    wave::ConfigParam<std::string> str_param{"string", &s, false};
    parser.loadParam(str_param);
    ASSERT_EQ("hello world!", s);
}

TEST(Utils_config_ConfigParser, loadArray) {
    std::vector<bool> b_array;
    std::vector<int> i_array;
    std::vector<float> f_array;
    std::vector<double> d_array;
    std::vector<std::string> s_array;
    wave::ConfigParser parser;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);
    parser.config_loaded = true;

    // BOOL ARRAY
    wave::ConfigParam<std::vector<bool>> b_param{"bool_array", &b_array};
    parser.loadParam(b_param);
    ASSERT_TRUE(b_array[0]);
    ASSERT_FALSE(b_array[1]);
    ASSERT_TRUE(b_array[2]);
    ASSERT_FALSE(b_array[3]);

    // INTEGER
    wave::ConfigParam<std::vector<int>> i_param{"int_array", &i_array};
    parser.loadParam(i_param);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(i + 1, i_array[i]);
    }

    // FLOAT
    wave::ConfigParam<std::vector<float>> f_param{"float_array", &f_array};
    parser.loadParam(f_param);
    for (int i = 0; i < 4; i++) {
        ASSERT_FLOAT_EQ((i + 1) * 1.1, f_array[i]);
    }

    // DOUBLE
    wave::ConfigParam<std::vector<double>> d_param{"double_array", &d_array};
    parser.loadParam(d_param);
    for (int i = 0; i < 4; i++) {
        ASSERT_FLOAT_EQ((i + 1) * 1.1, d_array[i]);
    }

    // STRING
    wave::ConfigParam<std::vector<std::string>> s_param{"string_array",
                                                        &s_array};
    parser.loadParam(s_param);
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
    int res;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);
    parser.config_loaded = true;

    // VECTOR 2
    wave::ConfigParam<wave::Vec2> vec2_param{"vector2", &vec2};
    res = parser.loadParam(vec2_param);

    EXPECT_EQ(0, res);
    ASSERT_FLOAT_EQ(1.1, vec2(0));
    ASSERT_FLOAT_EQ(2.2, vec2(1));

    // VECTOR 3
    wave::ConfigParam<wave::Vec3> vec3_param{"vector3", &vec3};
    res = parser.loadParam(vec3_param);

    EXPECT_EQ(0, res);
    ASSERT_FLOAT_EQ(1.1, vec3(0));
    ASSERT_FLOAT_EQ(2.2, vec3(1));
    ASSERT_FLOAT_EQ(3.3, vec3(2));

    // VECTOR 4
    wave::ConfigParam<wave::Vec4> vec4_param{"vector4", &vec4};
    res = parser.loadParam(vec4_param);

    EXPECT_EQ(0, res);
    ASSERT_FLOAT_EQ(1.1, vec4(0));
    ASSERT_FLOAT_EQ(2.2, vec4(1));
    ASSERT_FLOAT_EQ(3.3, vec4(2));
    ASSERT_FLOAT_EQ(4.4, vec4(3));

    // VECTOR X
    wave::ConfigParam<wave::VecX> vecx_param{"vector", &vecx};
    res = parser.loadParam(vecx_param);

    EXPECT_EQ(0, res);
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

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG);
    parser.config_loaded = true;

    // MATRIX 2
    wave::ConfigParam<wave::Mat2> mat2_param{"matrix2", &mat2};
    parser.loadParam(mat2_param);

    ASSERT_FLOAT_EQ(1.1, mat2(0, 0));
    ASSERT_FLOAT_EQ(2.2, mat2(0, 1));
    ASSERT_FLOAT_EQ(3.3, mat2(1, 0));
    ASSERT_FLOAT_EQ(4.4, mat2(1, 1));

    // MATRIX 3
    wave::ConfigParam<wave::Mat3> mat3_param{"matrix3", &mat3};
    parser.loadParam(mat3_param);

    index = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, mat3(i, j));
            index++;
        }
    }

    // MATRIX 4
    wave::ConfigParam<wave::Mat4> mat4_param{"matrix4", &mat4};
    parser.loadParam(mat4_param);

    index = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, mat4(i, j));
            index++;
        }
    }

    // MATRIX X
    wave::ConfigParam<wave::MatX> matx_param{"matrix", &matx};
    parser.loadParam(matx_param);
    index = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, matx(i, j));
            index++;
        }
    }

    // CV MATRIX
    wave::ConfigParam<cv::Mat> cvmat_param{"matrix", &cvmat};
    parser.loadParam(cvmat_param);

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
