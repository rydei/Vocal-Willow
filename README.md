# Vocal-Willow
Vocal Willow, an affordable, audio-first attachable sensor system designed to enable blind and visually impaired cricketers,PaddleBall and PickleBall players to practice independently. Developed specifically as an MVP for Samsung Solve for Tomorrow 2026, this project leverages affordable hardware and rule-based edge computing to bridge the independence gap in blind sports.

## Journey 
### Chapter-1(V1)
**Started on June 1*** with only one goal in mind to not stop till we land on a solid project.
We Spent like 5 days going back and fourth to double check and make sure the project we were going to work on was actually going to make an impact in the sports field.

**June 6**, We started working on V1 for our project. We knew the very first prototype would be massive as compared to the final product but we didnt expect it be as big as the size of an average digital clock.
Problems we ran in V1:-

-At first the question was how to power the whole Prototype up and since we didnt have any boost module or small lipo battery we used two !8650 li-ion batteries which was a bit bulky but it worked... atleast for a while before it would disconnect after every 2 minutes. The problem was with an overdischarged li-ion battery which was causing the issue though we figured it out pretty quickly.

-Another issue that we encountered was the mismatch of uuid between the website and prototype which had us in an uproar for quite a while (everyone makes a bit of silly mistakes!)
<img width="720" height="1252" alt="Screenshot_20260610_180928_Gallery" src="https://github.com/user-attachments/assets/65c11c16-2895-40a5-b7d2-cd8580481193" />

### Chapter-2(V2)
**June 12** (went on holidays for a few days between 6 and 12), With V1 finalized, now we knew how to scale it down while maintaining the core features of the project. We managed to scale it to 1/3rd the size of V1 by just using 3.7V LiPo Battery and a small boost module.It worked great till it didn't.

-After Leaving the module switched on for 3 hours it randomly died (it was supposed to last for 4.5 hours) and upon inspection we found out that the battery charging module was fried (TP4056) , a bit confused at first , we found out that the battery was overdischarged(again) but the TP4056 module had battery protection so im still kinda confused why it happened. Thought i managed to charge the battery back to normal voltage it didnt behave the same as the ESP32 started overheating when i connected it all back together with a new charging module.

<img width="900" height="1600" alt="WhatsApp Image 2026-06-18 at 10 18 52 PM (1) (1)" src="https://github.com/user-attachments/assets/57a826e2-bda2-4469-a8e9-1685087612f6" />


That's all for now. I plan on using 18650 again but a single cell this time so that the battery is easy to replace and can tolerate much higher temps.
