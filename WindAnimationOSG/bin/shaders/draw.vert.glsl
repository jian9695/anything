uniform sampler2D u_particles;
varying vec2 v_particle_pos;
varying vec2 v_tex_pos;
void main() {
    v_tex_pos = gl_Vertex.xy;
    v_particle_pos = texture2D(u_particles, gl_Vertex.xy).rg;
    gl_PointSize = 1.0;
    gl_Position = vec4(1.0 - 2.0 * v_particle_pos.x, 1.0 - 2.0 * v_particle_pos.y, 0, 1);
}
