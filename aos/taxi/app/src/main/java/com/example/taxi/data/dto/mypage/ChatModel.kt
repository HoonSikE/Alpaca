package com.example.taxi.data.dto.mypage

import kotlin.collections.HashMap

class ChatModel (val users: HashMap<String, Boolean> = HashMap(),
                 val comments : HashMap<String, CommentModel> = HashMap()){
    class CommentModel(val userName: String? = null, val message: String? = null, val time: String? = null)
}