/* tabs/payload.js — Payload configuration.
 * Owns firmware parameter group "PAYLOAD". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'payload', title: 'Payload', order: 90,
    render(el, ctx) {
      ctx.renderGroup(el, 'PAYLOAD', {
        title: 'Payload',
        hint: 'Camera pan/zoom servo rates and travel limits. The torch duty ceiling is a compile-time hard cap in payload.h and is deliberately not editable here.'.replace(/^'|'$/g, '')
      });
    }
  });
}
