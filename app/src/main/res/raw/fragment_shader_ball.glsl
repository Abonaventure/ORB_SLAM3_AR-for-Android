precision mediump float;
varying vec4 vPosition;//���մӶ�����ɫ�������Ķ���λ��
uniform sampler2D u_TextureUnit;      	 								
varying vec2 v_TextureCoordinates;
void main()                         
{
   float uR = 0.6;//��İ뾶
   vec4 color;
   float n = 4.0;//��Ϊn��n��n��
   float span = 2.0*uR/n;//�����γ���
   //�������в���
   int i = int((vPosition.x + uR)/span);//����
   int j = int((vPosition.y + uR)/span);//����
   int k = int((vPosition.z + uR)/span);//����
   int colorType = int(mod(float(i+j+k),2.0));
   if(colorType == 1) {//����ʱΪ��ɫ
   		color = vec4(0.2,1.0,0.129,0);
   }
   else {//ż��ʱΪ��ɫ
   		color = vec4(1.0,1.0,1.0,0);//��ɫ
   }
   //�����������ɫ����ƬԪ
   gl_FragColor=color;
   gl_FragColor = texture2D(u_TextureUnit, v_TextureCoordinates);  
}     