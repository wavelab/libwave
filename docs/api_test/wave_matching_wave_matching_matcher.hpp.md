## Classes

- `wave::Matcher`



### wave::Matcher

**Member Variables**:



**Methods**:

    const Eigen::MatrixXd & getInfo()


---

    float getRes()


---

    virtual bool match()


---

    const Eigen::Affine3d & getResult()


---

    void setup(const T &ref,
               const T &target)


---

    virtual void setTarget(const T &target)


---

    virtual void setRef(const T &ref)


---