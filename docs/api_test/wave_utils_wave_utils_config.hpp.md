## Enums

- `wave::ConfigDataType`


### wave::ConfigDataType



    TYPE_NOT_SET 0
    BOOL 1
    INT 2
    FLOAT 3
    DOUBLE 4
    STRING 5
    BOOL_ARRAY 11
    INT_ARRAY 12
    FLOAT_ARRAY 13
    DOUBLE_ARRAY 14
    STRING_ARRAY 15
    VEC2 21
    VEC3 22
    VEC4 23
    VECX 24
    MAT2 31
    MAT3 32
    MAT4 33
    MATX 34
    CVMAT 35




## Classes

- `wave::ConfigParam`
- `wave::ConfigParser`



### wave::ConfigParam

**Member Variables**:

    ConfigDataTypetype
    std::stringkey
    booloptional
    bool *b
    int *i
    float *f
    double *d
    std::string *s
    std::vector<bool> *b_array
    std::vector<int> *i_array
    std::vector<float> *f_array
    std::vector<double> *d_array
    std::vector<std::string> *s_array
    Vec2 *vec2
    Vec3 *vec3
    Vec4 *vec4
    VecX *vecx
    Mat2 *mat2
    Mat3 *mat3
    Mat4 *mat4
    MatX *matx
    cv::Mat *cvmat




### wave::ConfigParser

**Member Variables**:

    boolconfigured
    boolloaded
    YAML::Noderoot
    std::vector<ConfigParam>params


**Methods**:

    int getYamlNode(std::string key,
                    YAML::Node &node)


---

    int checkMatrix(std::string key,
                    bool optional)


---

    int checkKey(std::string key,
                 bool optional)


---

    void addParam(std::string key,
                  T *out,
                  bool optional)


---

    int loadPrimitive(ConfigParam param)


---

    int loadMatrix(ConfigParam param)


---

    int loadVector(ConfigParam param)


---

    int load(std::string config_file)


---

    int loadArray(ConfigParam param)


---

    int checkVector(std::string key,
                    enum ConfigDataType type,
                    bool optional)


---