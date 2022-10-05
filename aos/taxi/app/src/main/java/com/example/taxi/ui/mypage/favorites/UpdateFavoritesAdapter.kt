package com.example.taxi.ui.mypage.favorites

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.databinding.ItemFavoritesBinding

class UpdateFavoritesAdapter: RecyclerView.Adapter<UpdateFavoritesAdapter.FavoritesViewHolder>() {
    private var favoritesList = mutableListOf<Favorites>()
    lateinit var onItemClickListener: (View, String) -> Unit

    fun setListData(data: MutableList<Favorites>){
        favoritesList = data
    }

    fun updateList(list: MutableList<Favorites>){
        this.favoritesList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): FavoritesViewHolder {
        // recyclerview item 설정
        return FavoritesViewHolder(
            ItemFavoritesBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onItemClickListener)
        }
    }

    override fun onBindViewHolder(holder: FavoritesViewHolder, position: Int) {
        holder.bind(favoritesList[position])
    }

    override fun getItemCount(): Int {
        return favoritesList.size
    }

    class FavoritesViewHolder(private val binding: ItemFavoritesBinding) :
        RecyclerView.ViewHolder(binding.root) {
        lateinit var place : String
        private lateinit var address : String
        private lateinit var latitude : String
        private lateinit var longitude : String

        fun bind(data: Favorites) {
            binding.addrName.text = data.addressName
            place = data.addressName
            address = data.address
            latitude = data.latitude
            longitude = data.longitude
        }

        fun bindOnItemClickListener(onItemClickListener: (View, String) -> Unit ) {
            binding.imgStar.setOnClickListener {
                onItemClickListener(it, address)
            }
        }
    }
}