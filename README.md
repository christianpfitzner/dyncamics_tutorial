# dyncamics_tutorial

This is a small tutorial for a 2R robotics manipulator based on a simple Matlab/Octave implementation. 



The output of the tutorial is a plot, which shows the behaviour of two joints over time, including gravity, coriolis and mass moments. 


Here a starting angle of 0° and 0° is used, while both joints are enabled with a torque of 14 and 5 NM. 

![WhatsApp-Video-2023-05-10-at-10 48 16](https://github.com/christianpfitzner/dyncamics_tutorial/assets/20952014/a4ddd070-3bee-4122-9d01-482a31d93899)



## Dynamic Model

The mass matrix $M(\theta)$ is defined as:

$$
   M(\vec{\theta}) = \begin{pmatrix}
      m_1 L_1^2 + m_2(L_1^2 + 2 L_1 L_2 \cos \theta_2 + L_2^2) & m_2(L_1 L_2 \cos \theta_2 + L_2^2) \\
      m_2(L_1 L_2 \cos \theta_2 + L_2^2) & m_2 L_2^2 
   \end{pmatrix}
$$

The vector for coriolis and centripedal forces $c(\vec{\theta}, \dot{\vec{\theta}})$ is defined with:

$$
   c(\vec{\theta}, \dot{\vec{\theta}}) = \begin{pmatrix}
      -m_2 L_1 L_2 \sin \theta_2 (2 \dot{\theta_1} \dot{\theta_2} + \dot{\theta_2}^2) \\
      m_2 L_1 L_2 \dot{\theta_1} \sin \theta_2
   \end{pmatrix}
$$

The component moment for gravitiy $g(\vec{\theta})$ is defined as:

$$
   g(\vec{\theta}) = \begin{pmatrix}
      (m_1 + m_2 ) L_1 g \cos \theta_1 + m_2 L_2 g \cos (\theta_1 + \theta_2) \\
      m_2 g L_2  \cos (\theta_1 + \theta_2)
   \end{pmatrix}
$$


Those components form the equation of dynamics for a simplified 2R mainpulator

$$
      \vec{\tau} = M(\vec{\theta}) \ddot{\vec{\theta}} + C(\vec{\theta}, \dot{\vec{\theta}}) \dot{\vec{\theta}} + g(\vec{\theta}) - \vec{\tau}_\text{ext} \quad \text{,} 
$$



## Task: Implement the inverse dynamics model based on the provided template 
