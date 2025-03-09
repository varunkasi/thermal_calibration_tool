# Thermal calibration model
string model_type          # Type of model (e.g., "polynomial")
int8 degree                # Degree of polynomial (if applicable)
float32[] parameters       # Model parameters/coefficients
float32 r_squared          # R-squared value (coefficient of determination)
float32 rmse               # Root mean square error (in Celsius)
uint32 points_count        # Number of calibration points used
builtin_interfaces/Time timestamp  # When calibration was performed
uint16[] raw_value_range   # Min and max raw values covered [min, max]
