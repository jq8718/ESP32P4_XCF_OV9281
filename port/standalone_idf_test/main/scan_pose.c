#include "scan_pose.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "scan_pose"
#define CAL_MAGIC 0x504f5345u
#define CAL_VERSION 1u
#define RAD_TO_DEG 57.29577951308232f

typedef struct {
    uint32_t magic;
    uint32_t version;
    scan_camera_calibration_t calibration;
} stored_calibration_t;

static scan_camera_calibration_t s_calibration = {
    .width = 1280, .height = 720,
    .fx = 733.242515f, .fy = 732.511533f,
    .cx = 589.273600f, .cy = 359.376026f,
    .k1 = 0.119448f, .k2 = -0.223125f,
    .p1 = 0.000163f, .p2 = -0.000596f, .k3 = 0.083362f,
};
static bool s_nvs_ready;
static portMUX_TYPE s_calibration_lock = portMUX_INITIALIZER_UNLOCKED;

static bool calibration_valid(const scan_camera_calibration_t *c)
{
    if (!c || c->width != 1280 || c->height != 720 ||
        !isfinite(c->fx) || !isfinite(c->fy) || !isfinite(c->cx) || !isfinite(c->cy) ||
        c->fx < 100.0f || c->fx > 10000.0f || c->fy < 100.0f || c->fy > 10000.0f ||
        c->cx < 0.0f || c->cx >= c->width || c->cy < 0.0f || c->cy >= c->height) return false;
    const float d[] = {c->k1, c->k2, c->p1, c->p2, c->k3};
    for (unsigned i = 0; i < sizeof(d) / sizeof(d[0]); ++i)
        if (!isfinite(d[i]) || fabsf(d[i]) > 10.0f) return false;
    return true;
}

void scan_pose_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable (%s); using factory calibration in RAM", esp_err_to_name(err));
        return;
    }
    s_nvs_ready = true;
    nvs_handle_t handle;
    err = nvs_open("scan_pose", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "using factory OV9281 calibration; send @CAL,SET to save custom values");
        return;
    }
    stored_calibration_t stored;
    size_t size = sizeof(stored);
    err = nvs_get_blob(handle, "cal_v1", &stored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(stored) && stored.magic == CAL_MAGIC &&
        stored.version == CAL_VERSION && calibration_valid(&stored.calibration)) {
        portENTER_CRITICAL(&s_calibration_lock);
        s_calibration = stored.calibration;
        portEXIT_CRITICAL(&s_calibration_lock);
        ESP_LOGI(TAG, "loaded camera calibration from NVS");
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "stored camera calibration invalid; using factory values");
    }
}

void scan_pose_get_calibration(scan_camera_calibration_t *calibration)
{
    if (calibration) {
        portENTER_CRITICAL(&s_calibration_lock);
        *calibration = s_calibration;
        portEXIT_CRITICAL(&s_calibration_lock);
    }
}

esp_err_t scan_pose_set_calibration(const scan_camera_calibration_t *calibration)
{
    if (!calibration_valid(calibration)) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_ready) return ESP_ERR_INVALID_STATE;
    stored_calibration_t stored = {.magic = CAL_MAGIC, .version = CAL_VERSION,
                                   .calibration = *calibration};
    nvs_handle_t handle;
    esp_err_t err = nvs_open("scan_pose", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, "cal_v1", &stored, sizeof(stored));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_calibration_lock);
        s_calibration = *calibration;
        portEXIT_CRITICAL(&s_calibration_lock);
    }
    return err;
}

static bool solve8(float a[8][9], float x[8])
{
    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 8; ++row)
            if (fabsf(a[row][col]) > fabsf(a[pivot][col])) pivot = row;
        if (fabsf(a[pivot][col]) < 1e-8f) return false;
        if (pivot != col) for (int j = col; j < 9; ++j) {
            float t = a[col][j]; a[col][j] = a[pivot][j]; a[pivot][j] = t;
        }
        float divisor = a[col][col];
        for (int j = col; j < 9; ++j) a[col][j] /= divisor;
        for (int row = 0; row < 8; ++row) {
            if (row == col) continue;
            float factor = a[row][col];
            for (int j = col; j < 9; ++j) a[row][j] -= factor * a[col][j];
        }
    }
    for (int i = 0; i < 8; ++i) x[i] = a[i][8];
    return true;
}

static bool undistort(float xd, float yd, const scan_camera_calibration_t *c,
                      float *xu, float *yu)
{
    float x = xd, y = yd;
    for (int i = 0; i < 8; ++i) {
        float r2 = x*x + y*y, r4 = r2*r2, r6 = r4*r2;
        float radial = 1.0f + c->k1*r2 + c->k2*r4 + c->k3*r6;
        if (!isfinite(radial) || fabsf(radial) < 1e-5f) return false;
        float dx = 2.0f*c->p1*x*y + c->p2*(r2 + 2.0f*x*x);
        float dy = c->p1*(r2 + 2.0f*y*y) + 2.0f*c->p2*x*y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    *xu = x; *yu = y;
    return isfinite(x) && isfinite(y);
}

static float dot3(const float a[3], const float b[3])
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static float norm3(const float a[3]) { return sqrtf(dot3(a, a)); }
static void scale3(float a[3], float s) { a[0]*=s; a[1]*=s; a[2]*=s; }
static void cross3(const float a[3], const float b[3], float out[3])
{ out[0]=a[1]*b[2]-a[2]*b[1]; out[1]=a[2]*b[0]-a[0]*b[2]; out[2]=a[0]*b[1]-a[1]*b[0]; }

/* R is column-major. Optimize six physical pose parameters, rather than the
 * unconstrained eight homography coefficients. All residuals include the same
 * five-coefficient lens distortion model used by the PC's projectPoints(). */
typedef struct { float r[3][3], t[3]; } pose_candidate_t;

static float pose_residual(const pose_candidate_t *p, const float object[4][2],
                           const float source[4][2], const scan_camera_calibration_t *c,
                           float residual[8], float jacobian[8][6])
{
    float error = 0;
    for (int i = 0; i < 4; ++i) {
        float q[3];
        for (int j = 0; j < 3; ++j)
            q[j] = p->r[0][j]*object[i][0] + p->r[1][j]*object[i][1];
        float z = q[2] + p->t[2];
        if (z <= 1e-3f) return INFINITY;
        float x = (q[0]+p->t[0])/z, y = (q[1]+p->t[1])/z;
        float r2 = x*x+y*y, r4 = r2*r2;
        float radial = 1+c->k1*r2+c->k2*r4+c->k3*r4*r2;
        float xd = x*radial+2*c->p1*x*y+c->p2*(r2+2*x*x);
        float yd = y*radial+c->p1*(r2+2*y*y)+2*c->p2*x*y;
        float du = c->fx*xd-(source[i][0]-c->cx);
        float dv = c->fy*yd-(source[i][1]-c->cy);
        residual[2*i] = du; residual[2*i+1] = dv;
        error += du*du+dv*dv;
        if (!jacobian) continue;
        float slope = 2*(c->k1+2*c->k2*r2+3*c->k3*r4);
        float dxx = radial+x*x*slope+2*c->p1*y+6*c->p2*x;
        float dxy = x*y*slope+2*c->p1*x+2*c->p2*y;
        float dyy = radial+y*y*slope+6*c->p1*y+2*c->p2*x;
        float pixel[2][3] = {
            {c->fx*dxx/z, c->fx*dxy/z, -c->fx*(dxx*x+dxy*y)/z},
            {c->fy*dxy/z, c->fy*dyy/z, -c->fy*(dxy*x+dyy*y)/z}};
        for (int j = 0; j < 2; ++j) {
            float *a = pixel[j], *b = jacobian[2*i+j];
            /* d(R*P)/d(omega) = -[R*P]x. Scale translation increments
             * by depth so rotation and translation have comparable units. */
            b[0] = -a[1]*q[2]+a[2]*q[1];
            b[1] = a[0]*q[2]-a[2]*q[0];
            b[2] = -a[0]*q[1]+a[1]*q[0];
            for (int k = 0; k < 3; ++k) b[3+k] = a[k]*p->t[2];
        }
    }
    return isfinite(error) ? error : INFINITY;
}

static bool solve6(float a[6][7], float x[6])
{
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        for (int row = col+1; row < 6; ++row)
            if (fabsf(a[row][col]) > fabsf(a[pivot][col])) pivot = row;
        if (fabsf(a[pivot][col]) < 1e-12f) return false;
        for (int j = col; j < 7; ++j) {
            float temp = a[col][j]; a[col][j] = a[pivot][j]; a[pivot][j] = temp;
        }
        float divisor = a[col][col];
        for (int j = col; j < 7; ++j) a[col][j] /= divisor;
        for (int row = col+1; row < 6; ++row) {
            float factor = a[row][col];
            for (int j = col; j < 7; ++j) a[row][j] -= factor*a[col][j];
        }
    }
    for (int i = 5; i >= 0; --i) {
        x[i] = a[i][6];
        for (int j = i+1; j < 6; ++j) x[i] -= a[i][j]*x[j];
        if (!isfinite(x[i])) return false;
    }
    return true;
}

static void rotate_pose(pose_candidate_t *p, const float omega[3])
{
    float angle2 = dot3(omega, omega);
    float a = 1-angle2/6, b = .5f-angle2/24;
    if (angle2 > 1e-6f) {
        float angle = sqrtf(angle2);
        a = sinf(angle)/angle; b = (1-cosf(angle))/angle2;
    }
    for (int col = 0; col < 3; ++col) {
        float first[3], second[3];
        cross3(omega, p->r[col], first);
        cross3(omega, first, second);
        for (int j = 0; j < 3; ++j) p->r[col][j] += a*first[j]+b*second[j];
    }
}

static float refine_pose(pose_candidate_t *p, const float object[4][2],
                         const float source[4][2], const scan_camera_calibration_t *c)
{
    float residual[8], jacobian[8][6];
    float cost = pose_residual(p, object, source, c, residual, jacobian);
    float damping = 1e-4f;
    /* Fixed work bound; no allocation, no frame history, no smoothing. */
    for (int iteration = 0; iteration < 40 && isfinite(cost); ++iteration) {
        float normal[6][7] = {{0}}, step[6];
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 8; ++k) normal[i][j] += jacobian[k][i]*jacobian[k][j];
            for (int k = 0; k < 8; ++k) normal[i][6] -= jacobian[k][i]*residual[k];
            normal[i][i] += damping*fmaxf(normal[i][i], 1e-6f);
        }
        if (!solve6(normal, step)) break;
        pose_candidate_t trial = *p;
        rotate_pose(&trial, step);
        for (int j = 0; j < 3; ++j) trial.t[j] += step[3+j]*p->t[2];
        float trial_residual[8];
        float next = pose_residual(&trial, object, source, c, trial_residual, NULL);
        if (next < cost) {
            float improvement = cost-next;
            *p = trial;
            cost = next;
            float step_norm = 0;
            for (int j = 0; j < 6; ++j) step_norm += step[j]*step[j];
            if (improvement < 1e-8f || step_norm < 1e-12f) break;
            damping = fmaxf(damping*.25f, 1e-7f);
            pose_residual(p, object, source, c, residual, jacobian);
        } else {
            damping *= 10;
            if (damping > 1e6f) break;
        }
    }
    return cost;
}

static bool alternate_pose(const pose_candidate_t *seed, pose_candidate_t *alternate)
{
    /* A small planar target can have two plausible tilts. Reflect the normal
     * around the centre viewing ray to seed the other basin, refine both, and
     * choose the smaller pixel error (not whichever was found first). */
    float ray[3] = {seed->t[0], seed->t[1], seed->t[2]};
    scale3(ray, 1/norm3(ray));
    float reflected[3], axis[3];
    float projection = dot3(ray, seed->r[2]);
    for (int i = 0; i < 3; ++i) reflected[i] = 2*projection*ray[i]-seed->r[2][i];
    cross3(seed->r[2], reflected, axis);
    float sine = norm3(axis), cosine = dot3(seed->r[2], reflected);
    if (sine < 1e-5f) return false;
    scale3(axis, atan2f(sine, cosine)/sine);
    *alternate = *seed;
    rotate_pose(alternate, axis);
    return true;
}

bool scan_pose_estimate(const XcrBarcodeResult *result, scan_pose_result_t *pose)
{
    if (!result || !pose || !result->geometry_valid || result->geometry_quality != 2 ||
        result->length < 1 || (result->data[0] != '0' && result->data[0] != '1')) return false;
    scan_camera_calibration_t c;
    scan_pose_get_calibration(&c);
    if (!calibration_valid(&c)) return false;
    const float side = result->data[0] == '0' ? 30.0f : 50.0f;
    const float half = side * 0.5f;
    const float object[4][2] = {{-half, half}, {half, half}, {half, -half}, {-half, -half}};
    float image[4][2], source[4][2];
    uint64_t started = esp_timer_get_time();
    for (int i = 0; i < 4; ++i) {
        float u = result->subpixel_valid ? result->subpixel_corners[i*2] : (float)result->corners[i*2];
        float v = result->subpixel_valid ? result->subpixel_corners[i*2+1] : (float)result->corners[i*2+1];
        if (!isfinite(u) || !isfinite(v) || u < 0 || v < 0 || u >= c.width || v >= c.height) return false;
        source[i][0] = u; source[i][1] = v;
        if (!undistort((u-c.cx)/c.fx, (v-c.cy)/c.fy, &c, &image[i][0], &image[i][1])) return false;
    }

    float equations[8][9] = {{0}};
    for (int i = 0; i < 4; ++i) {
        float X = object[i][0], Y = object[i][1], x = image[i][0], y = image[i][1];
        int r = i*2;
        equations[r][0]=X; equations[r][1]=Y; equations[r][2]=1;
        equations[r][6]=-x*X; equations[r][7]=-x*Y; equations[r][8]=x;
        equations[r+1][3]=X; equations[r+1][4]=Y; equations[r+1][5]=1;
        equations[r+1][6]=-y*X; equations[r+1][7]=-y*Y; equations[r+1][8]=y;
    }
    float h[8];
    if (!solve8(equations, h)) return false;
    float r1[3]={h[0], h[3], h[6]}, r2[3]={h[1], h[4], h[7]};
    float t[3]={h[2], h[5], 1.0f};
    float scale = 2.0f / (norm3(r1) + norm3(r2));
    if (!isfinite(scale) || scale <= 0) return false;
    if (t[2] * scale < 0) scale = -scale;
    scale3(r1, scale); scale3(r2, scale); scale3(t, scale);
    float n1 = norm3(r1);
    if (n1 < 1e-6f) return false;
    scale3(r1, 1.0f/n1);
    float projection = dot3(r1, r2);
    for (int i = 0; i < 3; ++i) r2[i] -= projection*r1[i];
    float n2 = norm3(r2);
    if (n2 < 1e-6f) return false;
    scale3(r2, 1.0f/n2);
    float r3[3]; cross3(r1, r2, r3);
    if (t[2] <= 0 || !isfinite(t[0]) || !isfinite(t[1]) || !isfinite(t[2])) return false;

    pose_candidate_t refined = {
        .r = {{r1[0],r1[1],r1[2]}, {r2[0],r2[1],r2[2]}, {r3[0],r3[1],r3[2]}},
        .t = {t[0],t[1],t[2]}};
    pose_candidate_t alternate;
    float squared_error = refine_pose(&refined, object, source, &c);
    /* Recheck the winning normal once when a noisy homography started far
     * from either minimum. Usually the first reflection already suffices. */
    for (int pass = 0; pass < 2 && alternate_pose(&refined, &alternate); ++pass) {
        float alternate_error = refine_pose(&alternate, object, source, &c);
        if (alternate_error < squared_error) {
            refined = alternate;
            squared_error = alternate_error;
        } else {
            break;
        }
    }
    if (!isfinite(squared_error) || sqrtf(squared_error*0.25f) > 5.0f) return false;
    memcpy(r1, refined.r[0], sizeof(r1));
    memcpy(r2, refined.r[1], sizeof(r2));
    memcpy(r3, refined.r[2], sizeof(r3));
    memcpy(t, refined.t, sizeof(t));

    /* R columns map the physical code axes to camera axes. */
    float m02 = r3[0], m12 = r3[1], m22 = r3[2];
    float m10 = r1[1], m11 = r2[1];
    pose->pitch_x_deg = asinf(fmaxf(-1.0f, fminf(1.0f, m12))) * RAD_TO_DEG;
    pose->yaw_y_deg = atan2f(-m02, -m22) * RAD_TO_DEG;
    pose->roll_z_deg = atan2f(m10, -m11) * RAD_TO_DEG;
    pose->range_m = norm3(t) / 1000.0f;
    pose->optical_depth_m = t[2] / 1000.0f;
    pose->distance_to_plane_m = -(r3[0]*t[0] + r3[1]*t[1] + r3[2]*t[2]) / 1000.0f;
    pose->tx_m = t[0] / 1000.0f;
    pose->compute_us = (float)(esp_timer_get_time() - started);
    pose->reprojection_px = sqrtf(squared_error*.25f);
    return isfinite(pose->yaw_y_deg) && isfinite(pose->pitch_x_deg) &&
           isfinite(pose->roll_z_deg) && isfinite(pose->range_m) &&
           isfinite(pose->optical_depth_m) && isfinite(pose->distance_to_plane_m) &&
           isfinite(pose->tx_m) && pose->range_m > 0 && pose->distance_to_plane_m > 0;
}
