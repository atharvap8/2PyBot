/* tabs/yaw.js — Yaw / Steer configuration.
 * Owns firmware parameter group "YAW". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'yaw', title: 'Yaw / Steer', order: 40,
    render(el, ctx) {
      ctx.renderGroup(el, 'YAW', {
        title: 'Yaw / Steer',
        hint: 'Differential steering and encoder heading hold. YAWKP/YAWKD act on encoder COUNTS, so they must be rescaled whenever the wheel radius changes.'.replace(/^'|'$/g, '')
      });
    }
  });
}
