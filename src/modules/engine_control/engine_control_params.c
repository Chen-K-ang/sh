/**
 *
 *
 * _ECTL_TRIM
 *
 * @unit norm
 * @min -1.0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_TRIM, -0.42f); 

/**
 * 
 *
 * RAMP_TIME
 *
 * @unit norm
 * @min 0
 * @decimal 1
 * @increment 0.1
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(RAMP_TIME, 10.0f);

/**
 * 
 * _ECTL_P
 *
 * @unit norm
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_P, 0.30f);

/**
 * 
 * _ECTL_I
 *
 * @unit norm
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_I, 0.00f);

/**
 * 
 * _ECTL_D
 *
 * @unit norm
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_D, 0.00f);

/**
 * 
 * _ECTL_DOFFSET
 *
 * @unit norm
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_DOFFSET, 0.10f);

/**
 * 
 * _ECTL_SACL_UP
 *
 * @unit norm
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group Engine Control
 */
PARAM_DEFINE_FLOAT(_ECTL_SACL_UP, 2.40f);