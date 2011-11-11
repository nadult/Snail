uniform sampler3D volume;
varying vec3 texCoord;

void main()
{
	vec2 tex = texture3D(volume, texCoord).xy;
	float v = tex.x * (1.0 / 255.0) + tex.y * (254.0 / 255.0);
//	v *= 1.0f / 2.0f;
	gl_FragColor = vec4(v, v, v, 0.05);
}

