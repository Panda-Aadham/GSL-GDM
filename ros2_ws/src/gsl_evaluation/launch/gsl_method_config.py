"""Method catalogue and default parameter values for the VGR GSL wrapper."""

NON_SEMANTIC_METHODS = {
    "PMFS",
    "GrGSL",
    "ParticleFilter",
    "Spiral",
    "SurgeCast",
    "SurgeSpiral",
}

SEMANTIC_METHODS = {
    "SemanticPMFS",
    "SemanticGrGSL",
}


BASE_DEFAULTS = {
    "scale": 3,
    "markers_height": 0.2,
    "useDiffusionTerm": True,
    "stdevHit": 1.0,
    "stdevMiss": 1.2,
    "headless": False,
    "distanceWeight": 0.08,
    "openMoveSetExpasion": 5,
    "explorationProbability": 0.15,
    "maxUpdatesPerStop": 5,
    "kernelSigma": 1.5,
    "kernelStretchConstant": 1.5,
    "hitPriorProbability": 0.3,
    "confidenceSigmaSpatial": 1.0,
    "confidenceMeasurementWeight": 1.0,
    "useWindGroundTruth": True,
    "stepsSourceUpdate": 3,
    "maxRegionSize": 5,
    "refineFraction": 0.1,
    "blurSigmaX": 1.5,
    "blurSigmaY": 1.5,
    "step": 0.5,
    "initial_step": 0.6,
    "step_increment": 0.3,
    "Kmu": 0.5,
    "Kp": 1.0,
    "intervalLength": 0.5,
    "initSpiralStep": 1.0,
    "spiralStep_increment": 0.4,
    "numberOfParticles": 500,
    "maxEstimations": 20,
    "numberOfWindObs": 30,
    "convergenceThr": 0.5,
    "deltaT": 1.0,
    "mu": 0.9,
    "Sp": 0.01,
    "Rconv": 0.5,
    "stop_and_measure_time": 0.4,
    "th_gas_present": 0.1,
    "th_wind_present": 0.02,
    "max_wait_for_gas_time": 10.0,
    "global_exploration_on_gas_timeout": False,
    "grgsl_global_move_fallback": False,
    "initialExplorationMoves": 4,
}


METHOD_OVERRIDES = {
    "GrGSL": {
        "default_use_infotaxis": False,
        "stop_and_measure_time": 1.0,
        # VGR filament playback can produce much weaker concentration values
        # than the original GrGSL demo world, so keep GrGSL sensitive by default.
        "th_gas_present": 0.001,
        "th_wind_present": 0.001,
        "max_wait_for_gas_time": 6.0,
        "global_exploration_on_gas_timeout": True,
        "grgsl_global_move_fallback": True,
        "useDiffusionTerm": False,
        "stdevMiss": 1.5,
        "step": 0.7,
    },
    "SurgeCast": {
        "step": 1.0,
    },
    "SurgeSpiral": {
        "step": 1.0,
        "initSpiralStep": 1.0,
        "spiralStep_increment": 0.4,
    },
    "Spiral": {
        "initial_step": 0.6,
        "step_increment": 0.3,
        "intervalLength": 0.5,
    },
    "ParticleFilter": {
        "step": 1.0,
        "numberOfParticles": 500,
        "convergenceThr": 0.5,
    },
}


def method_defaults(method):
    """Return launch/GSL defaults for a supported method."""
    defaults = BASE_DEFAULTS.copy()
    defaults.update({
        "default_use_infotaxis": method == "PMFS",
        "use_gsl_rviz": method in NON_SEMANTIC_METHODS,
    })
    defaults.update(METHOD_OVERRIDES.get(method, {}))
    return defaults
