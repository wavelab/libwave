// Define features
std::vector<Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
edge_high.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::HIGH_POS, &(param.edge_tol)});

edge_low.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

flat.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

edge_int_high.emplace_back(Criteria{Kernel::FOG, SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
edge_int_high.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
edge_int_high.emplace_back(Criteria{Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

edge_int_low.emplace_back(Criteria{Kernel::FOG, SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
edge_int_low.emplace_back(Criteria{Kernel::LOAM, SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
edge_int_low.emplace_back(Criteria{Kernel::RNG_VAR, SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

this->feature_definitions.emplace_back(FeatureDefinition{edge_high, ResidualType::PointToLine, &(param.n_edge)});
this->feature_definitions.emplace_back(FeatureDefinition{edge_low, ResidualType::PointToLine, &(param.n_edge)});
this->feature_definitions.emplace_back(FeatureDefinition{flat, ResidualType::PointToPlane, &(param.n_flat)});
this->feature_definitions.emplace_back(
        FeatureDefinition{edge_int_high, ResidualType::PointToLine, &(param.n_int_edge)});
this->feature_definitions.emplace_back(
        FeatureDefinition{edge_int_low, ResidualType::PointToLine, &(param.n_int_edge)});