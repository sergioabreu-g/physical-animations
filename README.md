# Physical Animations through Reinforcement Learning

https://user-images.githubusercontent.com/26823024/121518972-65f83a00-c9f1-11eb-8755-5d155c319a72.mp4

In most videogames, character animations are predefined, artists make them during the production process and so they stay, as an immutable series of movements that are played apart from the physics of the world they're in.

Physics-based animations try to solve that problem. They're made by artists alike, but inside the game they're played by characters whose bodies are being simulated with realistic physics. Namely, each part of the body is simulated as a different physical object, and then they're connected by joints that can apply forces, mimicking the behavior of a real body. Applying the right forces one can make those characters play the animations previously made by the artists. And so you can achieve characters that not only can their animations be modified by external perturbations, but they can also interact with their environment in a more realistic manner.

The movement attained with this technique provokes a sensation of realism and immersion in the player that would be impossible with traditional animation techniques. Nevertheless, physically simulating a character this way presents two big problems: balance and stability. If animations were just played as they were made, the character wouldn't be able to stand or it would lose its trajectory when facing even the smallest perturbation. This can be fixed by applying artificial forces that ensure a specific pose, but at the cost of losing much of this technique's realism.

A better solution is to use machine learning so the character can learn to balance applying forces at its joints, trying to reach a pose as close as possible to the animation. This is still a frontier in game development. In this project, we explore the use of reinforcement learning to solve the task of balance in physics-based animations. For that we use Unity, one of the most renowned videogame engines, with which we also aim to test the accessibility of these techniques to game studios as of today.
