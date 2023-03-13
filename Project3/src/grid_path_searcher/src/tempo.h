 namespace ompl
 {
     namespace base
     {
         class RealVectorStateSampler : public StateSampler
         {
         public:
             RealVectorStateSampler(const StateSpace *space) : StateSampler(space)
             {
             }
  
             void sampleUniform(State *state) override;
             void sampleUniformNear(State *state, const State *near, double distance) override;
             void sampleGaussian(State *state, const State *mean, double stdDev) override;
         };
  
         class RealVectorStateSpace : public StateSpace
         {
         public:
             class StateType : public State
             {
             public:
                 StateType() = default;
  
                 double operator[](unsigned int i) const
                 {
                     return values[i];
                 }
  
                 double &operator[](unsigned int i)
                 {
                     return values[i];
                 }
  
                 double *values;
             };
  
             RealVectorStateSpace(unsigned int dim = 0)
               : dimension_(dim), bounds_(dim), stateBytes_(dim * sizeof(double))
             {
                 type_ = STATE_SPACE_REAL_VECTOR;
                 setName("RealVector" + getName());
                 dimensionNames_.resize(dim, "");
             }
  
             ~RealVectorStateSpace() override = default;
  
             void addDimension(double minBound = 0.0, double maxBound = 0.0);
  
             void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);
  
             void setBounds(const RealVectorBounds &bounds);
  
             void setBounds(double low, double high);
  
             const RealVectorBounds &getBounds() const
             {
                 return bounds_;
             }
  
             unsigned int getDimension() const override;
  
             const std::string &getDimensionName(unsigned int index) const;
  
             int getDimensionIndex(const std::string &name) const;
  
             void setDimensionName(unsigned int index, const std::string &name);
  
             double getMaximumExtent() const override;
  
             double getMeasure() const override;
  
             void enforceBounds(State *state) const override;
  
             bool satisfiesBounds(const State *state) const override;
  
             void copyState(State *destination, const State *source) const override;
  
             unsigned int getSerializationLength() const override;
  
             void serialize(void *serialization, const State *state) const override;
  
             void deserialize(State *state, const void *serialization) const override;
  
             double distance(const State *state1, const State *state2) const override;
  
             bool equalStates(const State *state1, const State *state2) const override;
  
             void interpolate(const State *from, const State *to, double t, State *state) const override;
  
             StateSamplerPtr allocDefaultStateSampler() const override;
  
             State *allocState() const override;
  
             void freeState(State *state) const override;
  
             double *getValueAddressAtIndex(State *state, unsigned int index) const override;
  
             void printState(const State *state, std::ostream &out) const override;
  
             void printSettings(std::ostream &out) const override;
  
             void registerProjections() override;
  
             void setup() override;
  
         protected:
             unsigned int dimension_;
  
             RealVectorBounds bounds_;
  
             std::vector<std::string> dimensionNames_;
  
             std::map<std::string, unsigned int> dimensionIndex_;
  
         private:
             std::size_t stateBytes_;
         };
     }
 }
  
 #endif