//vertex_shader_ball.glsl
uniform mat4 u_Matrix;//���յı任����
attribute vec4 a_Position;//����λ��
varying vec4 vPosition;//���ڴ��ݸ�ƬԪ��ɫ���Ķ���λ��
attribute vec2 a_TextureCoordinates;
varying vec2 v_TextureCoordinates;
void main()                    
{                              
    gl_Position = u_Matrix * a_Position;
    vPosition = a_Position;
    v_TextureCoordinates = a_TextureCoordinates;	 
} 