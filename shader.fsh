uniform sampler3D volume;
varying vec3 texCoord;

void main()
{
	vec2 tex = texture3D(volume, texCoord).xy;
	float v = tex.y;//(tex.x * (1.0 / 255.0) + tex.y * (254.0 / 255.0));
	float dist = distance(texCoord.xy, vec2(0.5, 0.5));

	if(dist > 0.497)
		v = 0.0;
	v = v * 10.0;

	gl_FragColor = vec4(1.0, 1.0, 1.0, v * 0.5);
}

