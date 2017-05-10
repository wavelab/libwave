## Classes

- `wave::Matcher`



### wave::Matcher

**Member Variables**:



**Methods**:

    void setup(const T &ref,
               const T &target)


---

    float getRes()


---

    virtual bool match()


---

    virtual void setTarget(const T &target)


---

    const Eigen::Affine3d & getResult()


---

    const Eigen::MatrixXd & getInfo()


---

    virtual void setRef(const T &ref)


---